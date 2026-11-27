#include "rigExec/internal/weightField.h"
// The baked program's pose half: the interleaved solver/constraint walk, at
// bake and at run, plus the rest->pose matrices and the frame publication it
// feeds.
// Pose-domain bake and frame path; must agree on captured vs re-read state.
// The evaluator is not visible here: everything this file needs of it was
// captured into RigExecBakedProgramImpl at Build (see bakedProgramImpl.h),
// and the compiled walk arrives restated as RigExecBakedWalkEntry.
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/bakedSlotDomains.h"
#include "rigExec/internal/bakedExecCrossCheckRows.h"
#include "bakedProgramImpl.h"
#include "bakedExecCrossCheckRows.h"
#include "poseEvalHost.h"

#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/frozenContextInternal.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/solverKernels.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExecScene/sceneCompileInputs.h"
#include "rigExecScene/solverSceneTranslation.h"
#include "rigExecScene/skeleton2dTranslation.h"
#include "rigExecScene/startFrameDerivation.h"
#include "rigExecScene/constraintSceneTranslation.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/api/types.h"

#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/base/work/threadLimits.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace rigExec {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((rigExecJoints, SchemaAttrText::rigExecJoints))
    ((rigExecJointElements, SchemaAttrText::rigExecJointElements))
    ((rigExecControlSpace, SchemaAttrText::rigExecControlSpace))
    ((rigExecControls, SchemaAttrText::rigExecControls))
    ((rigExecStartFrame, SchemaAttrText::rigExecStartFrame))
    ((rigExecRootControl, SchemaAttrText::rigExecRootControl))
    ((rigExecEffectorControl, SchemaAttrText::rigExecEffectorControl))
    ((rigExecPoleControl, SchemaAttrText::rigExecPoleControl))
    ((rigExecMidControl, SchemaAttrText::rigExecMidControl))
    ((rigExecEndControl, SchemaAttrText::rigExecEndControl))
    ((rigExecSpaceMatrix, SchemaAttrText::rigExecSpaceMatrix))
    ((rigExecSpace, SchemaAttrText::rigExecSpace))
    ((rigExecPreferredBendRadians, SchemaAttrText::rigExecPreferredBendRadians))
    ((rigExecUpperLengthOffset, SchemaAttrText::rigExecUpperLengthOffset))
    ((rigExecLowerLengthOffset, SchemaAttrText::rigExecLowerLengthOffset))
    ((rigExecStretchPolicy, SchemaAttrText::rigExecStretchPolicy))
    ((rigExecSegmentScale, SchemaAttrText::rigExecSegmentScale))
    ((rigExecScaleCalibration, SchemaAttrText::rigExecScaleCalibration))
    ((rigExecInputA, SchemaAttrText::rigExecInputA))
    ((rigExecInputB, SchemaAttrText::rigExecInputB))
    ((rigExecScaleBlend, SchemaAttrText::rigExecScaleBlend))
    ((rigExecRotationBlend, SchemaAttrText::rigExecRotationBlend))
    ((rigExecVolumeWeights, SchemaAttrText::rigExecVolumeWeights))
    ((rigExecRestLength, SchemaAttrText::rigExecRestLength))
    ((rigExecRootTangent, SchemaAttrText::rigExecRootTangent))
    ((rigExecStart, SchemaAttrText::rigExecStart))
    ((rigExecEnd, SchemaAttrText::rigExecEnd))
    ((rigExecWeights, SchemaAttrText::rigExecWeights))
    ((rigExecCount, SchemaAttrText::rigExecCount))
    ((rigExecDriverCurve, SchemaAttrText::rigExecDriverCurve))
    ((rigExecEndFrame, SchemaAttrText::rigExecEndFrame))
    ((rigExecTwistFrames, SchemaAttrText::rigExecTwistFrames))
    ((rigExecSampleCount, SchemaAttrText::rigExecSampleCount))
    ((rigExecOrientationMode, SchemaAttrText::rigExecOrientationMode))
    ((rigExecSolverMode, SchemaAttrText::rigExecSolverMode))
    ((rigExecPoleVectorMode, SchemaAttrText::rigExecPoleVectorMode))
    ((rigExecEvaluationMode, SchemaAttrText::rigExecEvaluationMode))
    ((rigExecRotationOrder, SchemaAttrText::rigExecRotationOrder))
    ((rigExecIkValue, SchemaAttrText::rigExecIkValue))
    ((rigExecWaves, SchemaAttrText::rigExecWaves))
    ((rigExecWaveLength, SchemaAttrText::rigExecWaveLength))
    ((rigExecFrequency, SchemaAttrText::rigExecFrequency))
    ((rigExecIgnoreFirstFrame, SchemaAttrText::rigExecIgnoreFirstFrame))
    ((rigExecMatchVelocity, SchemaAttrText::rigExecMatchVelocity))
    ((rigExecAimAxis, SchemaAttrText::rigExecAimAxis))
    ((rigExecWorldUpType, SchemaAttrText::rigExecWorldUpType))
    ((rigExecSources, SchemaAttrText::rigExecSources))
    ((inputsStretch, SchemaAttrText::inputsStretch))
    ((inputsSoftness, SchemaAttrText::inputsSoftness))
    ((inputsPin, SchemaAttrText::inputsPin))
    ((inputsSoftDistance, SchemaAttrText::inputsSoftDistance))
    ((inputsTwist, SchemaAttrText::inputsTwist))
    ((inputsUpperScale, SchemaAttrText::inputsUpperScale))
    ((inputsLowerScale, SchemaAttrText::inputsLowerScale))
    ((inputsSwitch, SchemaAttrText::inputsSwitch))
    ((inputsWeight, SchemaAttrText::inputsWeight))
    ((inputsPreserveVolume, SchemaAttrText::inputsPreserveVolume))
    ((inputsMidFollowWeight, SchemaAttrText::inputsMidFollowWeight))
    ((inputsRoll, SchemaAttrText::inputsRoll))
    ((inputsMinLengthRatio, SchemaAttrText::inputsMinLengthRatio))
    ((inputsTwistTurns, SchemaAttrText::inputsTwistTurns))
    ((inputsEnabled, SchemaAttrText::inputsEnabled))
    ((inputsDefaultWeight, SchemaAttrText::inputsDefaultWeight))
    ((inputsSourceWeights, SchemaAttrText::inputsSourceWeights))
    ((inputsTranslationOffsets, SchemaAttrText::inputsTranslationOffsets))
    ((inputsRotationOffsets, SchemaAttrText::inputsRotationOffsets))
    ((inputsPoleVector, SchemaAttrText::inputsPoleVector))
    ((inputsTwistDegrees, SchemaAttrText::inputsTwistDegrees))
    ((inputsPoleVectorWeights, SchemaAttrText::inputsPoleVectorWeights))
    ((inputsAmplitude, SchemaAttrText::inputsAmplitude))
    ((inputsDecay, SchemaAttrText::inputsDecay))
    ((inputsTermination, SchemaAttrText::inputsTermination))
    ((inputsAxisAmplitude, SchemaAttrText::inputsAxisAmplitude))
    ((inputsAffectTranslationX, SchemaAttrText::inputsAffectTranslationX))
    ((inputsAffectTranslationY, SchemaAttrText::inputsAffectTranslationY))
    ((inputsAffectTranslationZ, SchemaAttrText::inputsAffectTranslationZ))
    ((inputsAffectRotationX, SchemaAttrText::inputsAffectRotationX))
    ((inputsAffectRotationY, SchemaAttrText::inputsAffectRotationY))
    ((inputsAffectRotationZ, SchemaAttrText::inputsAffectRotationZ))
    ((inputsAffectScaleX, SchemaAttrText::inputsAffectScaleX))
    ((inputsAffectScaleY, SchemaAttrText::inputsAffectScaleY))
    ((inputsAffectScaleZ, SchemaAttrText::inputsAffectScaleZ))
    ((inputsTranslationOffset, SchemaAttrText::inputsTranslationOffset))
    ((inputsRotationOffset, SchemaAttrText::inputsRotationOffset))
    ((inputsScaleOffset, SchemaAttrText::inputsScaleOffset))
    ((inputsAimVector, SchemaAttrText::inputsAimVector))
    ((inputsUpVector, SchemaAttrText::inputsUpVector))
    ((inputsWorldUpVector, SchemaAttrText::inputsWorldUpVector))
    ((posedSpace, SchemaAttrText::posedSpace))
    ((avarsTx, SchemaAttrText::avarsTx))
    ((avarsTy, SchemaAttrText::avarsTy))
    ((avarsTz, SchemaAttrText::avarsTz))
    ((avarsSx, SchemaAttrText::avarsSx))
    ((avarsSy, SchemaAttrText::avarsSy))
    ((avarsSz, SchemaAttrText::avarsSz))
    (shortestArc)
    (aimX)
    (rotatePlane)
    (vector)
    (neverTS)
    ((xyz, SchemaValueText::xyz))
    (softDistance)
    (x)
    (none)
);

namespace {

/// The int \p path holds as Build reads it: a standing interactive override,
/// else an admitted upstream value, else \p authored.
int
_StandingInt(const RigExecBakedProgramImpl &B, const SdfPath &path,
             int authored)
{
    if (B.interactiveOverrides)
        for (const RigExecValueOverride &o : *B.interactiveOverrides)
            if (o.prim == path.GetPrimPath() &&
                o.attribute == path.GetNameToken() && o.value.IsHolding<int>())
                return o.value.UncheckedGet<int>();
    const auto upstream = B.upstream.find(path);
    if (upstream != B.upstream.end() && upstream->second.IsHolding<int>())
        return upstream->second.UncheckedGet<int>();
    return authored;
}

} // namespace

bool
RigExecBakedSolverCountsStand(const RigExecBakedProgramImpl &B)
{
    for (const RigExecBakedProgramImpl::Solver &s : B.solvers)
        if (s.kind == RigExecSolverKind::Ribbon &&
            RigExecBakedLeafRead(B, s.ribbonSampleCount) != s.ribbonSamples)
            return false;
    return true;
}

// Bake.

void
RigExecBakedBuildWalk(RigExecBakedBuildContext *ctx,
                      const std::vector<RigExecBakedWalkEntry> &walk)
{
    RigExecBakedProgramImpl &B = *ctx->program;
    const int N = int(B.paths.size());
    auto refuse = [&](const std::string &what, const SdfPath &where) {
        ctx->Refuse(what, where);
    };
    auto fold = [&](const UsdPrim &prim, const TfToken &name) {
        ctx->Fold(prim, name);
    };
    auto foldShape = [&](const UsdPrim &prim, const TfToken &name) {
        ctx->FoldShape(prim, name);
    };
    const RigExecSceneCompileInputs compileInputs(*B.sceneDescriptors);
    const auto readFact = [&](const SdfPath &path, UsdTimeCode identity,
                              auto *value) {
        RigExecSceneBoundInput input;
        return compileInputs.Bind(path, RigExecSceneReadRoute::Raw, &input) &&
               compileInputs.Read(input, identity, value);
    };
    auto readToken = [&](const UsdPrim &prim, const TfToken &name,
                         const TfToken &fallback) {
        TfToken value(fallback);
        readFact(prim.GetPath().AppendProperty(name), UsdTimeCode::Default(),
                 &value);
        return value;
    };
    auto targets = [&](const UsdPrim &prim, const TfToken &name) {
        return compileInputs.Targets(prim.GetPath().AppendProperty(name));
    };
    auto bind = [&](const UsdPrim &prim, const TfToken &name, auto fallback) {
        return ctx->Bind(prim, name, fallback);
    };
    auto slotOf = [&](const SdfPath &path) { return ctx->SlotOf(path); };
    RigExecPathText pathText;
    // "Publishes computePointFrame and computeRestFrame", which is what a
    // solver relationship target has to do for its frame to reach the
    // computation at all: the two are registered for RigExecJoint,
    // RigExecControl and the volume weights, and for nothing else. An
    // xform-derived slot is in the table so a constraint can name it, but
    // the walk seeds it from the stage and it declares no computation, so a
    // solver naming one reads a null pointer and publishes nothing.
    auto providerSlot = [&](const SdfPath &path) {
        const int slot = slotOf(path);
        return slot >= 0 && B.slotKind[size_t(slot)] ==
                                RigExecBakedSlotKind::FirstFramePose
                   ? slot
                   : -1;
    };

    std::map<SdfPath, RigExecPointFrame> providerRests;
    for (size_t slot = 0; slot < B.paths.size(); ++slot)
        if (B.slotKind[slot] == RigExecBakedSlotKind::FirstFramePose)
            providerRests.emplace(B.paths[slot], B.restFrames[slot]);

    auto bakeSolver = [&](const SdfPath &solverPath,
                          const std::vector<std::pair<SdfPath, int>>
                              &jointOutputs,
                          const std::vector<char> &snapshotJoints,
                          const SdfPathVector &liveRestJoints) {
        RigExecBakedProgramImpl::Solver s;
        // The first reason the solver cannot solve; Build refuses with it.
        std::string degenerateWhy;
        const auto degenerate = [&s, &degenerateWhy](const char *why) {
            if (!s.degenerate)
                degenerateWhy = why;
            s.degenerate = true;
        };
        // Every slot whose REST this description measures from, recorded at
        // the point the rest is read.
        auto foldRest = [&s](int slot) {
            if (slot >= 0) {
                s.restSlots.push_back(slot);
            }
            return slot;
        };
        s.path = solverPath;
        s.pathText = pathText(solverPath);
        const UsdPrim prim = B.stage->GetPrimAtPath(solverPath);
        const auto *node = compileInputs.Node(solverPath);
        s.type = node ? node->fact.type : TfToken();

        // rigExec:joints is the single declaration of the chain: a solver
        // whose output is consumed downstream claims no joints but still
        // names them for their REST measurements.
        std::vector<std::pair<int, int>> restRefs;
        {
            const SdfPathVector joints = targets(prim, _tokens->rigExecJoints);
            VtIntArray elements;
            fold(prim, _tokens->rigExecJointElements);
            readFact(solverPath.AppendProperty(_tokens->rigExecJointElements),
                     UsdTimeCode::Default(), &elements);
            s.execJointElements = elements;
            // exec binds ONE rest input per rigExec:joints target that
            // PUBLISHES computeRestFrame -- a target that publishes none
            // contributes no input at all -- and the computation then
            // compares the AUTHORED jointElements length against THAT count
            // and indexes the remap by position within it (the TwoBoneIk and
            // SplineIk rest loops in computations.cpp). Resolving against the
            // relationship's target count instead would remap by the wrong
            // index, and would keep remapping where exec gives up.
            // A FirstFramePose slot is what "publishes computeRestFrame" means: an
            // xform-derived slot exists in the table so a constraint can name
            // it, but exec seeds it from the stage and it declares no rest
            // computation, so it contributes no input here either.
            std::vector<int> restSlots;
            for (const SdfPath &joint : joints) {
                const int slot = slotOf(joint);
                if (slot >= 0 && B.slotKind[size_t(slot)] ==
                                     RigExecBakedSlotKind::FirstFramePose) {
                    restSlots.push_back(slot);
                }
            }
            // Only the two computations that DECLARE jointElements read it
            // (computations.cpp's TwoBoneIk and SplineIk input lists); for
            // FkChain and BlendPointFrames the attribute is not an input at
            // all, so an authored one -- whatever its length -- changes
            // nothing about what exec publishes and must not be allowed to
            // make this solver degenerate.
            const bool consumesElements =
                s.type == SchemaTypeName::RigExecTwoBoneIk ||
                s.type == SchemaTypeName::RigExecSplineIk;
            const bool remapped = consumesElements && !elements.empty();
            if (remapped && elements.size() != restSlots.size())
                degenerate("rigExec:jointElements does not give one element "
                           "per bound joint");
            const bool useRemap = remapped && !s.degenerate;
            for (size_t k = 0; k < restSlots.size(); ++k) {
                restRefs.emplace_back(restSlots[k],
                                      useRemap ? elements[k] : int(k));
            }
            s.restRefs = restRefs;
            // The LIVE half: a rest ref whose joint a pose step BELOW this
            // solver already wrote is measured from the frame that step left,
            // not from the authored rest. bindSolverReads binds it to a
            // version: the frame is not known until the run reaches this place.
            std::set<int> liveSlots;
            for (const SdfPath &joint : liveRestJoints) {
                const int slot = slotOf(joint);
                if (slot >= 0)
                    liveSlots.insert(slot);
            }
            s.restIsLive.assign(restRefs.size(), false);
            for (size_t k = 0; k < restRefs.size(); ++k) {
                if (liveSlots.count(restRefs[k].first)) {
                    s.restIsLive[k] = true;
                    s.hasLiveRest = true;
                }
            }
            s.restReads.assign(restRefs.size(), 0u);
            // The output basis, captured like every other rest description.
            // Folded into restSlots so a ladder recompute -- or a drag on one
            // of these rests -- dirties this solver exactly as a control rest
            // does.
            s.jointRests.reserve(restRefs.size());
            for (const auto &[slot, element] : restRefs) {
                s.jointRests.push_back(B.restPts[foldRest(slot)]);
            }
        }
        // A 2D skeleton translates before its reads bind: its selectors and
        // outside parent spaces come from the translation.
        RigExecSceneSolverDescriptor descriptor;
        std::string translationError;
        int translated = -1; // -1 not yet, 0 refused, 1 translated
        int ribbonSamples = 0;
        for (size_t k = 0; k < jointOutputs.size(); ++k) {
            const auto &[joint, element] = jointOutputs[k];
            const int slot = slotOf(joint);
            if (slot < 0) {
                // Unreachable: these joints come from the compiler's joint
                // binding, so they are in _jointPaths, and IsBakeable
                // refuses a rig whose joint is not a seeded pose provider.
                // No element is extracted for it; not a refusal.
                continue;
            }
            s.outputs.emplace_back(slot, element);
            s.outputSnapshots.push_back(
                k < snapshotJoints.size() ? snapshotJoints[k] : 0);
        }

        if (s.type == SchemaTypeName::RigExecFkChain ||
            s.type == SchemaTypeName::RigExecSkeleton2d) {
            // rigExec:startFrame: the dynamic computation prepends the
            // provider's rest-to-pose delta as a synthetic element 0 and
            // drops its frame from the aggregate; a 2D skeleton solves in
            // its rest frame and composes back through its pose. The bake
            // records the slot and rest here (foldRest registers the
            // refresh/dirty channels like any other rest read); the Solve
            // arm replays the shape. An unwired target, a second target,
            // or a non-provider target is the computation's answer, not
            // a degenerate: no frame input, so the chain solves absolute.
            // With no authored target, rigExec:startFramePolicy names the
            // provider through the rule scene translation applies.
            auto startTargets = targets(prim, _tokens->rigExecStartFrame);
            SdfPath derivedStart;
            if (startTargets.empty() &&
                RigExecDeriveSceneStartFrame(*B.sceneDescriptors, solverPath,
                                             &derivedStart) ==
                    RigExecStartFrameDerivation::Derived) {
                startTargets.push_back(derivedStart);
            }
            if (!startTargets.empty()) {
                const int slot = providerSlot(startTargets[0]);
                if (slot >= 0) {
                    s.start = slot;
                    s.startRest = B.restPts[foldRest(slot)];
                }
            }
        }
        if (s.type == SchemaTypeName::RigExecFkChain) {
            s.parentRelative =
                readToken(prim, _tokens->rigExecControlSpace, TfToken()) ==
                SchemaValueText::parentRelative;
            // The chain is the FILTERED list. The computation reads its
            // controls through a read iterator over the relationship's
            // targeted objects, so a target that publishes no frame
            // contributes no input at all: the chain SHORTENS and every
            // later element renumbers, which is what `int(k) - 1` then
            // means by a parent. The outputs mapping is not filtered -- its
            // element indices are the compiler's, over the same shortened
            // aggregate.
            for (const SdfPath &t : targets(prim, _tokens->rigExecControls)) {
                const int slot = providerSlot(t);
                if (slot < 0) {
                    continue;
                }
                s.controls.push_back(slot);
                s.controlRests.push_back(B.restPts[foldRest(slot)]);
            }
        } else if (s.type == SchemaTypeName::RigExecTwoBoneIk) {
            const auto r = targets(prim, _tokens->rigExecRootControl);
            const auto e = targets(prim, _tokens->rigExecEffectorControl);
            const auto p = targets(prim, _tokens->rigExecPoleControl);
            s.root = r.empty() ? -1 : providerSlot(r[0]);
            s.end = e.empty() ? -1 : providerSlot(e[0]);
            s.pole = p.empty() ? -1 : providerSlot(p[0]);
            if (s.root < 0 || s.end < 0 || s.pole < 0)
                degenerate("two-bone IK needs root, effector and pole "
                           "controls that are pose providers");
            // Bone lengths are MEASURED from the rests of the joints this
            // solver binds, so all three chain slots have to be bound by
            // one. The computation returns empty on the first element out
            // of [0, 3) and again when the three are not all seen; the two
            // give the same aggregate, so one flag answers both.
            std::array<bool, 3> seen{false, false, false};
            for (const auto &[slot, element] : restRefs) {
                if (element < 0 || element >= 3) {
                    degenerate("two-bone IK joint element is outside [0, 3)");
                    continue;
                }
                s.ikRests[size_t(element)] = B.restPts[foldRest(slot)];
                seen[size_t(element)] = true;
            }
            if (!(seen[0] && seen[1] && seen[2]))
                degenerate("two-bone IK joints do not bind all three chain "
                           "slots");
            // The bone lengths exec measures every evaluation, measured once.
            s.upperLengthBase = (s.ikRests[1][0] - s.ikRests[0][0]).GetLength();
            s.lowerLengthBase = (s.ikRests[2][0] - s.ikRests[1][0]).GetLength();
            s.ikSpace =
                bind(prim, _tokens->rigExecSpaceMatrix, GfMatrix4d(1.0));
            const auto sp = targets(prim, _tokens->rigExecSpace);
            s.spaceSlot = sp.empty() ? -1 : providerSlot(sp[0]);
            if (s.spaceSlot >= 0) {
                s.spaceRest = B.restPts[foldRest(s.spaceSlot)];
            }
            s.bend = bind(prim, _tokens->rigExecPreferredBendRadians, 0.0);
            s.upperOffset = bind(prim, _tokens->rigExecUpperLengthOffset, 0.0);
            s.lowerOffset = bind(prim, _tokens->rigExecLowerLengthOffset, 0.0);
            s.stretch = bind(prim, _tokens->inputsStretch, 1.0f);
            s.softness = bind(prim, _tokens->inputsSoftness, 0.0f);
            s.pin = bind(prim, _tokens->inputsPin, 0.0f);
            s.softDistance = bind(prim, _tokens->inputsSoftDistance, 0.0f);
            s.limbTwist = bind(prim, _tokens->inputsTwist, 0.0f);
            s.upperScale = bind(prim, _tokens->inputsUpperScale, 1.0);
            s.lowerScale = bind(prim, _tokens->inputsLowerScale, 1.0);
            s.ikParams.softDistancePolicy =
                readToken(prim, _tokens->rigExecStretchPolicy, TfToken()) ==
                SchemaValueText::softDistance;
            s.ikParams.scaleSegments =
                readToken(prim, _tokens->rigExecSegmentScale, TfToken()) ==
                SchemaValueText::toChild;
            fold(prim, _tokens->rigExecScaleCalibration);
            readFact(
                solverPath.AppendProperty(_tokens->rigExecScaleCalibration),
                UsdTimeCode::Default(), &s.ikParams.limb.scaleCalibration);
            s.ikParams.preferredBendRadians = s.bend.constant;
            s.ikParams.stretch = s.stretch.constant;
            s.ikParams.softness = s.softness.constant;
            RigExecTwoBoneIkLengths(
                s.ikRests, s.ikSpace.constant, s.upperOffset.constant,
                s.lowerOffset.constant, &s.ikParams.upperLength,
                &s.ikParams.lowerLength);
        } else if (s.type == SchemaTypeName::RigExecBlendPointFrames) {
            // Aggregate inputs bind after every solver descriptor exists.
            // Missing inputs retain the kernel's null-input behavior.
            auto solverSlot = [&](const SdfPathVector &v) {
                if (v.empty())
                    return -1;
                const auto it = B.solverIndex.find(v[0]);
                if (it != B.solverIndex.end())
                    return it->second;
                return -1;
            };
            s.inA = solverSlot(targets(prim, _tokens->rigExecInputA));
            s.inB = solverSlot(targets(prim, _tokens->rigExecInputB));
            s.blendWeight = bind(prim, _tokens->inputsWeight, 0.0f);
            s.scaleMode = readToken(prim, _tokens->rigExecScaleBlend,
                                    TfToken()) == SchemaValueText::linear
                              ? RigExecScaleBlend::Linear
                              : RigExecScaleBlend::Log;
            // The fallback is the only mode there is, so an unauthored
            // token is never rejected; anything else is refused at Build
            // once both inputs bind solvers. A blend with one input unbound
            // passes the other through and never reads the token.
            if (readToken(prim, _tokens->rigExecRotationBlend,
                          _tokens->shortestArc) !=
                SchemaValueText::shortestArc) {
                s.blendRotationRejected = true;
            }
        } else if (s.type == SchemaTypeName::RigExecSplineIk) {
            const auto r = targets(prim, _tokens->rigExecRootControl);
            const auto m = targets(prim, _tokens->rigExecMidControl);
            const auto e = targets(prim, _tokens->rigExecEndControl);
            s.root = r.empty() ? -1 : providerSlot(r[0]);
            s.mid = m.empty() ? -1 : providerSlot(m[0]);
            s.end = e.empty() ? -1 : providerSlot(e[0]);
            if (s.root < 0 || s.mid < 0 || s.end < 0)
                degenerate("spline IK needs root, mid and end controls that "
                           "are pose providers");
            const size_t count = restRefs.size();
            if (count == 0)
                degenerate("spline IK rigExec:joints binds no joints");
            s.splineCount = count;
            std::vector<RigExecPointFrame> restJoints(count);
            s.splineJointRests.resize(count);
            // The remap must be a PERMUTATION of the chain slots: a slot
            // filled twice leaves another empty, and the computation refuses
            // to build a rest curve out of that ("chain slot %d is filled
            // twice") rather than solving against a hole.
            std::vector<bool> filled(count, false);
            for (const auto &[slot, element] : restRefs) {
                if (element < 0 || size_t(element) >= count ||
                    filled[size_t(element)]) {
                    degenerate("spline IK rigExec:jointElements is not a "
                               "permutation of its chain");
                    continue;
                }
                filled[size_t(element)] = true;
                restJoints[size_t(element)] = B.restFrames[foldRest(slot)];
                s.splineJointRests[size_t(element)] = B.restPts[slot];
            }
            std::vector<double> weights;
            VtFloatArray authoredWeights;
            fold(prim, _tokens->rigExecVolumeWeights);
            readFact(solverPath.AppendProperty(_tokens->rigExecVolumeWeights),
                     UsdTimeCode::Default(), &authoredWeights);
            s.execSplineWeights = authoredWeights;
            for (float w : authoredWeights)
                weights.push_back(w);
            if (!weights.empty() && weights.size() != count)
                degenerate("spline IK rigExec:volumeWeights length differs "
                           "from its joints");
            const TfToken restTok =
                readToken(prim, _tokens->rigExecRestLength, TfToken());
            if (!restTok.IsEmpty() && restTok != SchemaValueText::curve &&
                restTok != SchemaValueText::chain)
                degenerate("spline IK rigExec:restLength is neither curve nor "
                           "chain");
            // Exec rebuilds this description every evaluation. It is a pure
            // function of the rests plus these two, so the two are kept and
            // the rebuild happens again only on a run that recomposed them.
            s.splineRestWeights = weights;
            s.splineRestMode = restTok == SchemaValueText::chain
                                   ? RigExecSplineIkRestLength::Chain
                                   : RigExecSplineIkRestLength::Curve;
            const auto splineSpaceTargets =
                targets(prim, _tokens->rigExecSpace);
            s.spaceSlot = splineSpaceTargets.empty()
                              ? -1
                              : providerSlot(splineSpaceTargets[0]);
            if (s.spaceSlot >= 0) {
                s.spaceRest = B.restPts[foldRest(s.spaceSlot)];
            }
            s.splineRestFrames = restJoints;
            s.splineRootRest = s.root >= 0 ? B.restFrames[foldRest(s.root)]
                                           : RigExecPointFrame();
            s.splineMidRest = s.mid >= 0 ? B.restFrames[foldRest(s.mid)]
                                         : RigExecPointFrame();
            s.splineEndRest = s.end >= 0 ? B.restFrames[foldRest(s.end)]
                                         : RigExecPointFrame();
            s.splineRest = RigExecSplineIkMakeRest(
                restJoints,
                s.root >= 0 ? B.restFrames[foldRest(s.root)]
                            : RigExecPointFrame(),
                s.mid >= 0 ? B.restFrames[foldRest(s.mid)]
                           : RigExecPointFrame(),
                s.end >= 0 ? B.restFrames[foldRest(s.end)]
                           : RigExecPointFrame(),
                weights,
                restTok == SchemaValueText::chain
                    ? RigExecSplineIkRestLength::Chain
                    : RigExecSplineIkRestLength::Curve);
            s.preserveVolume = bind(prim, _tokens->inputsPreserveVolume, 1.0);
            s.midFollowWeight = bind(prim, _tokens->inputsMidFollowWeight, 0.5);
            s.roll = bind(prim, _tokens->inputsRoll, 0.0);
            s.twist = bind(prim, _tokens->inputsTwist, 0.0);
            s.minLengthRatio = bind(prim, _tokens->inputsMinLengthRatio, 0.0);
            s.splineParamsVary = s.preserveVolume.varying ||
                                 s.midFollowWeight.varying || s.roll.varying ||
                                 s.twist.varying || s.minLengthRatio.varying;
            s.splineParams.preserveVolume = s.preserveVolume.constant;
            s.splineParams.midFollowWeight = s.midFollowWeight.constant;
            s.splineParams.roll = GfDegreesToRadians(s.roll.constant);
            s.splineParams.twist = GfDegreesToRadians(s.twist.constant);
            s.splineParams.minLengthRatio = s.minLengthRatio.constant;
            const TfToken tangent =
                readToken(prim, _tokens->rigExecRootTangent, TfToken());
            if (tangent == SchemaValueText::aim) {
                s.splineParams.aimRootTangent = true;
            } else if (!tangent.IsEmpty() &&
                       tangent != SchemaValueText::rigid) {
                degenerate("spline IK rigExec:rootTangent is neither aim nor "
                           "rigid");
            }
        } else if (s.type == SchemaTypeName::RigExecTwistDistribution) {
            // The two endpoint frames arrive on the batch request as FINAL
            // frames, the same way an FkChain's controls do, so they are
            // provider slots read out of `fin`.
            // A target that is no provider slot, or one the exec network
            // seeds from the stage rather than computing (an xform-derived
            // slot declares neither computePointFrame nor computeRestFrame),
            // leaves the computation's REQUIRED start or end input unbound,
            // and Build refuses the solver.
            const auto endpoint = [&](const SdfPathVector &v) {
                return v.empty() ? -1 : providerSlot(v[0]);
            };
            s.root = endpoint(targets(prim, _tokens->rigExecStart));
            s.end = endpoint(targets(prim, _tokens->rigExecEnd));
            if (s.root < 0 || s.end < 0) {
                degenerate("twist distribution needs start and end frames "
                           "that are pose providers");
            } else {
                // The rests are OPTIONAL inputs of the computation, but
                // every seeded provider publishes computeRestFrame, so a
                // resolved endpoint always has one and the identity
                // substitution exec makes for a missing one is unreachable
                // from here.
                s.twistStartRest = B.restPts[size_t(foldRest(s.root))];
                s.twistEndRest = B.restPts[size_t(foldRest(s.end))];
            }
            // rigExec:weights and rigExec:count define the frame cardinality,
            // which compile refuses to let vary with time, so both are read
            // once and folded. The float -> double widening is element-wise
            // and in array order, as the computation's read iterator does it.
            VtFloatArray authoredWeights;
            fold(prim, _tokens->rigExecWeights);
            readFact(solverPath.AppendProperty(_tokens->rigExecWeights),
                     UsdTimeCode::Default(), &authoredWeights);
            s.execTwistWeights = authoredWeights;
            for (float w : authoredWeights)
                s.twistWeights.push_back(w);
            int count = 1;
            fold(prim, _tokens->rigExecCount);
            readFact(solverPath.AppendProperty(_tokens->rigExecCount),
                     UsdTimeCode::Default(), &count);
            s.execTwistCount = count;
            RigExecResolveTwistWeights(count, &s.twistWeights);
            s.twistTurns = bind(prim, _tokens->inputsTwistTurns, 0.0);
        } else if (s.type == SchemaTypeName::RigExecRibbon) {
            // The driver curve's points, resolved by the compiler to the exact
            // native attribute (a prim target becomes its .points). Not read
            // through bind(): neither a connection on that attribute nor the
            // generation's resolved inputs may answer for it.
            const auto found = ctx->ribbonDriverPoints.find(solverPath);
            if (found != ctx->ribbonDriverPoints.end()) {
                s.ribbonPointsPath = found->second;
            }
            std::string phaseError;
            if (!RigExecResolveReadPhase(
                    prim.GetRelationship(_tokens->rigExecDriverCurve),
                    &s.ribbonPointsPhase, &phaseError))
                B.crossDomainErrors.push_back(phaseError);
            if (const UsdAttribute a =
                    B.stage->GetAttributeAtPath(s.ribbonPointsPath)) {
                // Read at Default, and left empty when nothing answers
                // there: a curve carrying only time samples has no bind-time
                // value, and an empty rest is what makes the sampler publish
                // nothing at all.
                VtVec3fArray rest;
                readFact(s.ribbonPointsPath, UsdTimeCode::Default(), &rest);
                s.ribbonRestPoints.assign(rest.cbegin(), rest.cend());
                s.ribbonPointsVarying =
                    a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0;
                if (s.ribbonPointsVarying) {
                    s.ribbonPointsQuery = UsdAttributeQuery(a);
                } else {
                    // One value at every time code, so the prologue has
                    // nothing to read and the cone nothing to compare.
                    VtVec3fArray live;
                    readFact(s.ribbonPointsPath, ctx->capture, &live);
                    s.ribbonConstantPoints.assign(live.cbegin(), live.cend());
                }
                // Registered BY PATH rather than through fold(prim, name): the
                // driver curve is another prim entirely, and the target may be
                // an arbitrary property path. Folded: a value edit on the curve
                // rebuilds, and an override on it cannot be placed. Unresolved
                // paths are not registered; property creation recompiles.
                B.rebuild.insert(s.ribbonPointsPath);
                B.folded.insert(s.ribbonPointsPath);
                B.named.insert(s.ribbonPointsPath);
                B.prims.insert(s.ribbonPointsPath.GetPrimPath());
            }
            // The count fixes the frame cardinality, so it is a Build fact.
            // It stays bound so a drag on it places; the run whose sample
            // differs from Build's rebuilds (RigExecBakedSolverCountsStand).
            s.ribbonSampleCount = bind(prim, _tokens->rigExecSampleCount, 5);
            ribbonSamples = _StandingInt(
                B, solverPath.AppendProperty(_tokens->rigExecSampleCount),
                s.ribbonSampleCount.constant);
            // rigExec:parameterization, frameTransport, startFrame,
            // endFrame and twistFrames are not read: they affect batching and
            // the signature only.
        } else if (s.type == SchemaTypeName::RigExecSkeleton2d) {
            translated = RigExecDispatchTranslateSceneSolver(
                             *B.sceneDescriptors, solverPath, providerRests,
                             &descriptor, &translationError)
                             ? 1
                             : 0;
            // Thirteen channels per bone, then the switch selectors.
            // Avars stay per-frame reads: a value edit patches the avar slot.
            for (const SdfPath &joint : targets(prim, _tokens->rigExecJoints)) {
                const UsdPrim bone = B.stage->GetPrimAtPath(joint);
                for (size_t c = 0; c < kRigExecBone2dChannelCount; ++c) {
                    const TfToken &name = RigExecBone2dChannelNames[c];
                    if (RigExecBone2dChannelIsBool(c)) {
                        s.bone2dBools.push_back(ctx->Bind(bone, name, false));
                        continue;
                    }
                    SdfPathVector walk;
                    RigExecBakedInput<double> input = ctx->ResolveBind(
                        bone, name, kRigExecBone2dChannelDefaults[c], &walk);
                    if (c < 4 && !input.varying && input.head) {
                        input.varying = true;
                        input.query = UsdAttributeQuery();
                        input.resolvedAttr = input.head;
                    }
                    ctx->CommitBind(bone, name, &input, walk);
                    s.bone2dDoubles.push_back(std::move(input));
                }
            }
            if (translated == 1) {
                for (size_t g = 0; g < descriptor.bone2dSelectorReads.size();
                     ++g) {
                    const SdfPath &property =
                        descriptor.bone2dSelectorReads[g].consumer;
                    const UsdPrim owner =
                        B.stage->GetPrimAtPath(property.GetPrimPath());
                    const double fallback =
                        g < descriptor.bone2dSelectorFallbacks.size()
                            ? descriptor.bone2dSelectorFallbacks[g]
                            : 0.0;
                    SdfPathVector walk;
                    RigExecBakedInput<double> input = ctx->ResolveBind(
                        owner, property.GetNameToken(), fallback, &walk);
                    ctx->CommitBind(owner, property.GetNameToken(), &input,
                                    walk);
                    s.bone2dSelectors.push_back(std::move(input));
                }
                // Outside parent spaces ride on the shared controls.
                for (const SdfPath &space : descriptor.controls) {
                    const int slot = providerSlot(space);
                    if (slot < 0) {
                        translated = 0;
                        std::string spaceText;
                        if (!RigExecSpellPathText(space, &spaceText))
                            spaceText = space.GetAsString();
                        translationError =
                            s.pathText + ": the 2D parent space " + spaceText +
                            " is not a pose provider";
                        break;
                    }
                    s.controls.push_back(slot);
                    s.controlRests.push_back(B.restPts[foldRest(slot)]);
                }
            }
        }
        // What can move this description, asked once. `restsVary` is the
        // whole rest CHAIN of every slot it measured from -- a provider's
        // rest is its ancestors' rests as well -- and `restOverrides` is
        // every rest channel of that same chain, so that a drag on one
        // dirties this solver rather than leaving it solving against the
        // authored rest the drag is standing in for.
        {
            // A LIVE rest is by definition per-frame: it is this frame pose as
            // an earlier step left it, so the description can never be
            // concluded constant.
            s.restsVary = s.restsVary || s.hasLiveRest;
            std::set<int> chain;
            for (const int named : s.restSlots) {
                for (int slot = named; slot >= 0;
                     slot = B.parent[size_t(slot)]) {
                    if (!chain.insert(slot).second) {
                        break;
                    }
                }
                s.restsVary =
                    s.restsVary || B.restChainVaries[size_t(named)] != 0;
            }
            for (const int slot : chain) {
                const RigExecBakedProgramImpl::Ladder &ladder =
                    B.ladders[size_t(slot)];
                const RigExecBakedInput<double> *avars = ladder.restAvars;
                if (ladder.restSpace.overrideIndex >= 0) {
                    s.restOverrides.push_back(ladder.restSpace.overrideIndex);
                }
                for (int c = 0; c < 6; ++c) {
                    if (avars[c].overrideIndex >= 0) {
                        s.restOverrides.push_back(avars[c].overrideIndex);
                    }
                }
            }
            std::sort(s.restOverrides.begin(), s.restOverrides.end());
            s.restOverrides.erase(
                std::unique(s.restOverrides.begin(), s.restOverrides.end()),
                s.restOverrides.end());
        }
        if (translated < 0)
            translated = RigExecDispatchTranslateSceneSolver(
                             *B.sceneDescriptors, solverPath, providerRests,
                             &descriptor, &translationError)
                             ? 1
                             : 0;
        if (translated == 0)
            refuse(translationError, solverPath);
        else {
            // Native input binding selects live/rest SSA versions; the common
            // structural record owns the solver's schema interpretation.
            // The bake's own rest reads (the live and the folded ones) win
            // over the record's compile-time copies.
            auto &record = descriptor.record;
            record.degenerate = record.degenerate || s.degenerate;
            record.restIsLive = std::move(s.restIsLive);
            record.jointRests = std::move(s.jointRests);
            if (s.type == SchemaTypeName::RigExecSkeleton2d) {
                record.controlRests = std::move(s.controlRests);
                record.startRest = s.startRest;
            }
            static_cast<RigExecSolverRecord &>(s) = std::move(record);
            if (s.degenerate)
                refuse(degenerateWhy.empty()
                           ? std::string("solver inputs do not bind")
                           : degenerateWhy,
                       solverPath);
            if (s.type == SchemaTypeName::RigExecRibbon)
                s.ribbonSamples = ribbonSamples;
            for (const auto &read : descriptor.jointGroupSelectorReads) {
                const UsdPrim owner =
                    B.stage->GetPrimAtPath(read.consumer.GetPrimPath());
                SdfPathVector walk;
                RigExecBakedInput<double> input = ctx->ResolveBind(
                    owner, read.consumer.GetNameToken(), 0.0, &walk);
                ctx->CommitBind(owner, read.consumer.GetNameToken(), &input,
                                walk);
                s.jointGroupSelectors.push_back(std::move(input));
            }
        }
        const int slot = int(B.solvers.size());
        B.solverIndex[solverPath] = slot;
        B.solvers.push_back(std::move(s));
        return slot;
    };

    // One plain Xformable a constraint reads off the stage, registered once
    // however many bindings name it.
    // The ancestor list is pure namespace topology -- every slot that is a
    // STRICT prefix of the path, shallowest first -- so it is settled here;
    // WHICH of them the source rides is a per-frame question about frames
    // the step reads out of declared slots. The prim and its ancestors go
    // into the resync index only: the transform is re-read every frame, so
    // a value edit on it must not rebuild the program.
    auto nativeSource = [&](const SdfPath &path) {
        for (size_t k = 0; k < B.nativeSources.size(); ++k) {
            if (B.nativeSources[k].path == path) {
                return int(k);
            }
        }
        RigExecBakedProgramImpl::NativeXformSource source;
        source.path = path;
        for (int i = 0; i < N; ++i) {
            if (B.paths[size_t(i)] != path &&
                path.HasPrefix(B.paths[size_t(i)])) {
                source.ancestorSlots.push_back(i);
            }
        }
        for (SdfPath p = path; !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
             p = p.GetParentPath()) {
            B.prims.insert(p);
            if (p == B.assetRootPath) {
                break;
            }
        }
        B.nativeSources.push_back(std::move(source));
        return int(B.nativeSources.size()) - 1;
    };

    auto bakeConstraint = [&](const RigExecBakedConstraintSpec &fc) {
        RigExecBakedProgramImpl::Constraint c;
        c.path = fc.moverPath;
        c.pathText = pathText(fc.moverPath);
        c.type = fc.schemaType;
        c.singleChainIk =
            fc.schemaType == SchemaTypeName::RigExecSingleChainIkConstraint;
        const UsdPrim prim = B.stage->GetPrimAtPath(fc.moverPath);
        RigExecSceneConstraintDescriptor sceneConstraint;
        bool translatedConstraint = false;
        const auto *constraintNode = compileInputs.Node(fc.moverPath);
        // Pose MatrixMover has its own matrix record and numerical body;
        // the shared constraint descriptor covers the six constraint schemas.
        if (fc.schemaType != SchemaTypeName::RigExecMatrixMover &&
            constraintNode && constraintNode->fact.type == fc.schemaType) {
            std::string error;
            translatedConstraint = RigExecDispatchTranslateSceneConstraint(
                *B.sceneDescriptors, fc.moverPath, fc.targets, &sceneConstraint,
                &error);
            if (!translatedConstraint)
                refuse(error, fc.moverPath);
            else
                c.kernelRecord = sceneConstraint.record;
        }
        // Every target, in compiled order, with the read-phase membership
        // the dynamic recordFrame tests per call. A SingleChainIK's targets
        // ARE its joint chain (rigEvaluator's compile sets the two equal),
        // so this is also the chain the commit revises atomically.
        for (size_t k = 0; k < fc.targets.size(); ++k) {
            const int slot = slotOf(fc.targets[k]);
            if (slot < 0) {
                refuse("constraint target is not a seeded pose provider",
                       fc.targets[k]);
                continue;
            }
            c.targetSlots.push_back(slot);
            c.snapshotTargets.push_back(
                k < fc.snapshotTargets.size() ? fc.snapshotTargets[k] : 0);
        }
        c.target = c.targetSlots.empty() ? -1 : c.targetSlots[0];
        for (size_t k = 0; k < fc.sources.size(); ++k) {
            // resolveBinding's order, decided once: the walk's own frame
            // when it holds one -- which is every RigExec provider and every
            // plain Xformable some constraint targets -- and the stage
            // otherwise. A source that is itself an xform-derived target must
            // take the SLOT, or a constraint chained onto another
            // constraint's target would read the unrevised transform.
            const int slot = slotOf(fc.sources[k]);
            const SdfPath &xform =
                k < fc.sourceXforms.size() ? fc.sourceXforms[k] : SdfPath();
            const int native =
                slot < 0 && !xform.IsEmpty() ? nativeSource(xform) : -1;
            if (slot < 0 && native < 0) {
                refuse("constraint source is not a pose provider",
                       fc.sources[k]);
            }
            c.sources.push_back(slot);
            c.sourceNatives.push_back(native);
            c.sourcePaths.push_back(fc.sources[k]);
            c.sourcePathTexts.push_back(pathText(fc.sources[k]));
        }
        c.enabled = bind(prim, _tokens->inputsEnabled, true);
        c.defaultWeight = bind(prim, _tokens->inputsDefaultWeight, 1.0f);
        // The envelope object, resolved per frame by the weight field and not
        // captured here: what the bake records is only that the constraint
        // HAS one, and enough of the invalidation index to notice the object
        // changing. Every read the weight field makes goes through the
        // generation's resolved inputs, so the prim is routed rather than
        // bound -- an interactive override on any property of it reaches the
        // next generation with nothing to place.
        c.weightObject = fc.weightObject;
        c.weightScratch.reserve(1);
        if (!c.weightObject.IsEmpty()) {
            B.prims.insert(c.weightObject);
            B.resolvedRoutedPrims.insert(c.weightObject);
        }

        // inputs:sourceWeights and the parent offsets: re-read when the program
        // stamp, the stage edit serial or (for a varying read) the time moved
        // (ConstraintArrays). FoldShape: edits rebuild; overrides not
        // placeable.
        {
            RigExecBakedProgramImpl::ConstraintArrays arrays;
            arrays.prim = prim;
            arrays.path = fc.moverPath;
            arrays.pathText = c.pathText;
            const auto &names =
                RigExecBakedProgramImpl::ConstraintArrays::Names();
            for (size_t channel = 0; prim && channel < 4; ++channel) {
                arrays.attributes[channel] = prim.GetAttribute(names[channel]);
                arrays.keys[channel] =
                    prim.GetPath().AppendProperty(names[channel]);
            }
            arrays.sourceCount = c.sources.size();
            // An unconsumed table has the same neutral shape as a missing
            // authored array. Admitted bodies still validate and replace it
            // from the current raw samples before using any entry.
            arrays.weights.assign(arrays.sourceCount, 1.0);
            arrays.translationOffsets.assign(arrays.sourceCount, GfVec3d(0));
            arrays.rotationOffsets.assign(arrays.sourceCount, GfVec3d(0));
            arrays.parentOffsets =
                fc.schemaType == SchemaTypeName::RigExecParentConstraint;
            foldShape(prim, _tokens->inputsSourceWeights);
            if (prim.GetAttribute(_tokens->inputsSourceWeights)) {
                ++B.boundInputs;
            }
            if (arrays.parentOffsets) {
                for (const TfToken &name : {_tokens->inputsTranslationOffsets,
                                            _tokens->inputsRotationOffsets}) {
                    foldShape(prim, name);
                    if (prim.GetAttribute(name)) {
                        ++B.boundInputs;
                    }
                }
            }
            c.arrays = int(B.constraintArrays.size());
            B.constraintArrays.push_back(std::move(arrays));
        }
        // The GEOMETRY domain, decided once. The delta is measured against
        // the TARGET PRIM's authored transform, which is a stage read the
        // prologue makes; the prim goes into the resync index only, because
        // nothing about its value is captured.
        c.pointsTarget = fc.pointsTarget;
        if (!fc.pointsTarget.IsEmpty() && !fc.targets.empty()) {
            c.deltaBasePath = fc.targets[0];
            c.deltaBasePathText = pathText(fc.targets[0]);
            c.deltaBase = int(B.deltaBasePaths.size());
            B.deltaBasePaths.push_back(fc.targets[0]);
            B.prims.insert(fc.targets[0]);
        }
        // Three uniform tokens, read with a plain Get (no time), plus the two
        // bindings resolved beside them. The evaluation mode's "autoDetect" is
        // settled here because
        // _IkUsesAnimatedTs asks only structural questions -- a solver binding,
        // a time sample, a connection -- and the attributes it inspects are
        // folded for their SHAPE so that authoring one rebuilds the program
        // rather than silently changing the answer.
        if (c.singleChainIk) {
            c.stretch = bind(prim, _tokens->inputsStretch, 0.0f);
            c.preserveJointOrientation =
                readToken(prim, _tokens->rigExecOrientationMode,
                          _tokens->aimX) == "preserve";
            c.ikMode = readToken(prim, _tokens->rigExecSolverMode,
                                 _tokens->rotatePlane) == "singleChain"
                           ? RigExecSingleChainIkMode::SingleChain
                           : RigExecSingleChainIkMode::RotatePlane;
            c.poleModeObject = readToken(prim, _tokens->rigExecPoleVectorMode,
                                         _tokens->vector) == "object";
            const TfToken evaluationMode = readToken(
                prim, _tokens->rigExecEvaluationMode, _tokens->neverTS);
            c.useAnimatedTs =
                evaluationMode == "alwaysTS" ||
                (evaluationMode == "autoDetect" && fc.ikUsesAnimatedTs);
            c.ikRestLive = fc.ikRestLive;
            for (const SdfPath &joint : fc.ikChain) {
                const UsdPrim jointPrim = B.stage->GetPrimAtPath(joint);
                for (const TfToken &name :
                     {_tokens->posedSpace, _tokens->avarsTx, _tokens->avarsTy,
                      _tokens->avarsTz, _tokens->avarsSx, _tokens->avarsSy,
                      _tokens->avarsSz}) {
                    foldShape(jointPrim, name);
                }
            }
            const auto bindSource = [&](const SdfPath &path,
                                        const SdfPath &xform, int *slot,
                                        int *native) {
                *slot = path.IsEmpty() ? -1 : slotOf(path);
                *native =
                    *slot < 0 && !xform.IsEmpty() ? nativeSource(xform) : -1;
            };
            bindSource(fc.effector, fc.effectorXform, &c.effector,
                       &c.effectorNative);
            c.effectorPath = fc.effector;
            for (size_t k = 0; k < fc.poleObjects.size(); ++k) {
                int slot = -1, native = -1;
                bindSource(fc.poleObjects[k],
                           k < fc.poleObjectXforms.size()
                               ? fc.poleObjectXforms[k]
                               : SdfPath(),
                           &slot, &native);
                c.poleObjects.push_back(slot);
                c.poleObjectNatives.push_back(native);
            }
            // Bound only in RotatePlane mode, the only mode that reads them:
            // binding them everywhere would let an override on one place itself
            // on a solve that ignores it.
            if (c.ikMode == RigExecSingleChainIkMode::RotatePlane) {
                c.poleVector =
                    bind(prim, _tokens->inputsPoleVector, GfVec3d(0, 1, 0));
                c.twistDegrees = bind(prim, _tokens->inputsTwistDegrees, 0.0);
            }
            RigExecBakedProgramImpl::ConstraintArrays &arrays =
                B.constraintArrays[size_t(c.arrays)];
            arrays.readPole =
                c.ikMode == RigExecSingleChainIkMode::RotatePlane &&
                c.poleModeObject && !c.poleObjects.empty();
            arrays.poleCount = c.poleObjects.size();
            arrays.poleWeights.assign(arrays.poleCount, 1.0);
            if (arrays.readPole &&
                prim.GetAttribute(_tokens->inputsPoleVectorWeights)) {
                foldShape(prim, _tokens->inputsPoleVectorWeights);
                ++B.boundInputs;
            }
        }
        c.order = RigExecBakedParseEulerOrder(
            readToken(prim, _tokens->rigExecRotationOrder, _tokens->xyz));
        if (fc.schemaType == SchemaTypeName::RigExecIkFkMatchConstraint) {
            c.ikFkMatch = true;
            if (!c.targetSlots.empty() || c.sources.size() != 10 ||
                std::any_of(c.sources.begin(), c.sources.end(),
                            [](int slot) { return slot < 0; })) {
                refuse("IK/FK match needs its joints, IK root, effector and "
                       "five controls as pose providers",
                       fc.moverPath);
            }
            // The controls presented with each of the five: every control
            // under one, riding the nearest of them above it.
            for (int slot = 0; slot < N; ++slot) {
                if (B.slotKind[size_t(slot)] !=
                    RigExecBakedSlotKind::FirstFramePose)
                    continue;
                int rides = -1;
                size_t depth = 0;
                for (size_t k = 5; k < c.sources.size(); ++k) {
                    const SdfPath &control = B.paths[size_t(c.sources[k])];
                    if (slot == c.sources[k]) {
                        rides = -1;
                        break;
                    }
                    if (B.paths[size_t(slot)].HasPrefix(control) &&
                        control.GetPathElementCount() > depth) {
                        rides = int(k - 5);
                        depth = control.GetPathElementCount();
                    }
                }
                if (rides >= 0)
                    c.matchRiders.emplace_back(slot, rides);
            }
            // Each scalar is bound AT the attribute its connection walk ends
            // on, in that attribute's precision: a float read whose head is a
            // double reads the walk as doubles, which cannot read a float
            // channel. Unconnected, it reads its own value.
            const TfToken names[7] = {
                _tokens->inputsSwitch,       _tokens->inputsStretch,
                _tokens->inputsSoftDistance, _tokens->inputsUpperScale,
                _tokens->inputsLowerScale,   _tokens->inputsPin,
                _tokens->inputsTwist};
            static const double fallbacks[7] = {0, 1, 0, 1, 1, 0, 0};
            for (size_t k = 0; k < 7; ++k) {
                auto &scalar = c.matchScalars[k];
                UsdAttribute end = prim.GetAttribute(names[k]);
                std::set<SdfPath> visited;
                while (end && visited.insert(end.GetPath()).second) {
                    SdfPathVector connections;
                    end.GetConnections(&connections);
                    if (connections.size() != 1)
                        break;
                    const UsdAttribute next =
                        B.stage->GetAttributeAtPath(connections[0]);
                    if (!next)
                        break;
                    end = next;
                }
                scalar.isFloat =
                    end && end.GetTypeName() == SdfValueTypeNames->Float;
                const UsdPrim owner = end ? end.GetPrim() : prim;
                const TfToken name = end ? end.GetName() : names[k];
                if (scalar.isFloat) {
                    scalar.narrow = bind(owner, name, float(fallbacks[k]));
                } else {
                    scalar.wide = bind(owner, name, fallbacks[k]);
                }
            }
            // Uniform constants: authoring one rebuilds the program.
            for (const auto &[name, value] :
                 {std::make_pair(_tokens->rigExecIkValue, &c.matchIkValue),
                  std::make_pair(_tokens->rigExecScaleCalibration,
                                 &c.matchScaleCalibration),
                  std::make_pair(_tokens->rigExecUpperLengthOffset,
                                 &c.matchUpperOffset),
                  std::make_pair(_tokens->rigExecLowerLengthOffset,
                                 &c.matchLowerOffset)}) {
                fold(prim, name);
                readFact(prim.GetPath().AppendProperty(name),
                         UsdTimeCode::Default(), value);
            }
            c.matchIkValue =
                std::isfinite(c.matchIkValue) ? c.matchIkValue : 1.0;
            fold(prim, _tokens->rigExecStretchPolicy);
            c.matchStretchLimb = readToken(prim, _tokens->rigExecStretchPolicy,
                                           _tokens->softDistance) ==
                                 SchemaValueText::softDistance;
        }

        if (fc.schemaType == SchemaTypeName::RigExecHarmonicsConstraint) {
            c.harmonics = true;
            if (c.targetSlots.size() != 1) {
                refuse("harmonics constraint needs one pose-provider target",
                       fc.moverPath);
            }
            // Uniform settings: they fix the window, so authoring one
            // rebuilds the program.
            for (const auto &[name, value] :
                 {std::make_pair(_tokens->rigExecWaves,
                                 &c.harmonicsSettings.waves),
                  std::make_pair(_tokens->rigExecWaveLength,
                                 &c.harmonicsSettings.waveLength),
                  std::make_pair(_tokens->rigExecFrequency,
                                 &c.harmonicsSettings.frequency)}) {
                fold(prim, name);
                readFact(prim.GetPath().AppendProperty(name),
                         UsdTimeCode::Default(), value);
            }
            for (const auto &[name, value] :
                 {std::make_pair(_tokens->rigExecIgnoreFirstFrame,
                                 &c.harmonicsSettings.ignoreFirstFrame),
                  std::make_pair(_tokens->rigExecMatchVelocity,
                                 &c.harmonicsMatchVelocity)}) {
                fold(prim, name);
                readFact(prim.GetPath().AppendProperty(name),
                         UsdTimeCode::Default(), value);
            }
            if (!RigExecHarmonicsSettingsValid(c.harmonicsSettings)) {
                refuse("harmonics constraint window is invalid", fc.moverPath);
            }
            // Each scalar bound in the precision of the attribute its
            // connection walk ends on, as the IK/FK match binds its own.
            const TfToken names[3] = {_tokens->inputsAmplitude,
                                      _tokens->inputsDecay,
                                      _tokens->inputsTermination};
            const double fallbacks[3] = {1.0, 3.0, 0.0};
            for (size_t k = 0; k < 3; ++k) {
                auto &scalar = c.harmonicsScalars[k];
                UsdAttribute end = prim.GetAttribute(names[k]);
                std::set<SdfPath> visited;
                while (end && visited.insert(end.GetPath()).second) {
                    SdfPathVector connections;
                    end.GetConnections(&connections);
                    if (connections.size() != 1)
                        break;
                    const UsdAttribute next =
                        B.stage->GetAttributeAtPath(connections[0]);
                    if (!next)
                        break;
                    end = next;
                }
                scalar.isFloat =
                    end && end.GetTypeName() == SdfValueTypeNames->Float;
                const UsdPrim owner = end ? end.GetPrim() : prim;
                const TfToken name = end ? end.GetName() : names[k];
                if (scalar.isFloat) {
                    scalar.narrow = bind(owner, name, float(fallbacks[k]));
                } else {
                    scalar.wide = bind(owner, name, fallbacks[k]);
                }
            }
            c.offset = bind(prim, _tokens->inputsAxisAmplitude, GfVec3d(1.0));
            RigExecBakedProgramImpl::WindowedOp window;
            window.frames =
                uint32_t(RigExecHarmonicsWindowFrames(c.harmonicsSettings));
            window.path = fc.moverPath;
            c.window = int(B.windowedOps.size());
            B.windowedOps.push_back(std::move(window));
            c.harmonicsPositions.reserve(
                3 * (size_t(B.windowedOps.back().frames) + 1));
        }

        if (fc.schemaType == SchemaTypeName::RigExecPositionConstraint) {
            c.affectX = bind(prim, _tokens->inputsAffectTranslationX, true);
            c.affectY = bind(prim, _tokens->inputsAffectTranslationY, true);
            c.affectZ = bind(prim, _tokens->inputsAffectTranslationZ, true);
            c.offset =
                ctx->Bind(prim, _tokens->inputsTranslationOffset, GfVec3d(0),
                          /*sourceValue=*/true);
        } else if (fc.schemaType == SchemaTypeName::RigExecRotationConstraint) {
            c.affectX = bind(prim, _tokens->inputsAffectRotationX, true);
            c.affectY = bind(prim, _tokens->inputsAffectRotationY, true);
            c.affectZ = bind(prim, _tokens->inputsAffectRotationZ, true);
            c.offset = bind(prim, _tokens->inputsRotationOffset, GfVec3d(0));
        } else if (fc.schemaType == SchemaTypeName::RigExecScaleConstraint) {
            c.affectX = bind(prim, _tokens->inputsAffectScaleX, true);
            c.affectY = bind(prim, _tokens->inputsAffectScaleY, true);
            c.affectZ = bind(prim, _tokens->inputsAffectScaleZ, true);
            c.offset = bind(prim, _tokens->inputsScaleOffset, GfVec3d(0));
        } else if (fc.schemaType == SchemaTypeName::RigExecParentConstraint) {
            c.tX = bind(prim, _tokens->inputsAffectTranslationX, true);
            c.tY = bind(prim, _tokens->inputsAffectTranslationY, true);
            c.tZ = bind(prim, _tokens->inputsAffectTranslationZ, true);
            c.rX = bind(prim, _tokens->inputsAffectRotationX, true);
            c.rY = bind(prim, _tokens->inputsAffectRotationY, true);
            c.rZ = bind(prim, _tokens->inputsAffectRotationZ, true);
            // FBX disables scale by default; the false fallback is the
            // authored contract (schema.usda), not an oversight.
            c.sX = bind(prim, _tokens->inputsAffectScaleX, false);
            c.sY = bind(prim, _tokens->inputsAffectScaleY, false);
            c.sZ = bind(prim, _tokens->inputsAffectScaleZ, false);
        } else if (fc.schemaType == SchemaTypeName::RigExecAimConstraint) {
            c.affectX = bind(prim, _tokens->inputsAffectRotationX, true);
            c.affectY = bind(prim, _tokens->inputsAffectRotationY, true);
            c.affectZ = bind(prim, _tokens->inputsAffectRotationZ, true);
            const UsdAttribute aimAttr =
                prim.GetAttribute(_tokens->inputsAimVector);
            // Whether it is authored, not what it holds, is baked: an
            // authored aimVector is a per-frame read; an unauthored one is
            // the aimAxis constant, with no read or override of its own.
            const bool aimVectorAuthored =
                translatedConstraint
                    ? sceneConstraint.aimVectorAuthored
                    : aimAttr && aimAttr.HasAuthoredValueOpinion();
            if (aimVectorAuthored)
                c.aimVector =
                    bind(prim, _tokens->inputsAimVector, GfVec3d(1, 0, 0));
            foldShape(prim, _tokens->inputsAimVector);
            const TfToken axis =
                readToken(prim, _tokens->rigExecAimAxis, _tokens->x);
            if (!aimVectorAuthored)
                c.aimVector.constant =
                    translatedConstraint ? sceneConstraint.aimAxisFallback
                    : axis == SchemaValueText::y ? GfVec3d(0, 1, 0)
                    : axis == SchemaValueText::z ? GfVec3d(0, 0, 1)
                                                 : GfVec3d(1, 0, 0);
            c.upVector = bind(prim, _tokens->inputsUpVector, GfVec3d(0, 1, 0));
            c.rotationOffset =
                bind(prim, _tokens->inputsRotationOffset, GfVec3d(0));
            c.worldUpVector =
                bind(prim, _tokens->inputsWorldUpVector, GfVec3d(0, 1, 0));
            c.worldUpType =
                readToken(prim, _tokens->rigExecWorldUpType, _tokens->none);
            const std::string up = UsdGeomGetStageUpAxis(B.stage).GetString();
            c.sceneUp = (up == "Z" || up == SchemaValueText::z)
                            ? GfVec3d(0, 0, 1)
                            : GfVec3d(0, 1, 0);
            // The aimTarget/aimAxis contract preserves input up; FBX
            // WorldUpType=None is the distinct minimum-swing mode.
            c.preserveInputUp = targets(prim, _tokens->rigExecSources).empty();
            c.worldUpObjectNamed = !fc.worldUpObject.IsEmpty();
            c.worldUpPath = fc.worldUpObject;
            c.worldUpObject =
                c.worldUpObjectNamed ? slotOf(fc.worldUpObject) : -1;
            if (c.worldUpObjectNamed && c.worldUpObject < 0 &&
                !fc.worldUpXform.IsEmpty()) {
                c.worldUpNative = nativeSource(fc.worldUpXform);
            }
            if (c.worldUpObjectNamed && c.worldUpObject < 0 &&
                c.worldUpNative < 0) {
                refuse("aim world-up object is not a pose provider",
                       fc.worldUpObject);
            }
        }
        // rigExec:space, on the operators the compile reads it for. The
        // compile only keeps a provider here, so a named space that has no
        // slot is a bake-time contradiction, not a per-frame one.
        c.blendShear = fc.blendShear;
        c.worldUpRotationOnly = fc.worldUpRotationOnly;
        c.radialBlend = fc.radialBlend;
        if (!fc.space.IsEmpty()) {
            c.spaceSlot = slotOf(fc.space);
            if (c.spaceSlot < 0) {
                refuse("constraint space is not a pose provider", fc.space);
            }
        }
        if (translatedConstraint) {
            c.order = sceneConstraint.order;
            c.preserveJointOrientation =
                sceneConstraint.record.singleChain.preserveJointOrientation;
            c.ikMode = sceneConstraint.record.singleChain.mode;
            c.poleModeObject = sceneConstraint.poleModeObject;
            c.useAnimatedTs = sceneConstraint.useAnimatedTs;
            c.worldUpType = sceneConstraint.worldUpType;
            c.sceneUp = sceneConstraint.sceneUp;
            c.preserveInputUp = sceneConstraint.preserveInputUp;
            c.blendShear = sceneConstraint.blendShear;
            c.worldUpRotationOnly = sceneConstraint.worldUpRotationOnly;
        }
        c.kernelInputs.sources.reserve(c.sources.size());
        c.kernelInputs.chain.reserve(c.targetSlots.size());
        c.kernelResult.chain.reserve(c.targetSlots.size());
        B.constraints.push_back(std::move(c));
        return int(B.constraints.size()) - 1;
    };

    std::vector<bool> ownedBySolver(N, false);
    for (int i = 0; i < N; ++i) {
        ownedBySolver[i] = B.jointSolverBinding->count(B.paths[i]) > 0;
    }
    // Every provider inherits its namespace pose: an authored or connected
    // parent:space would have refused the bake above.
    auto buildPropagation = [&](const std::vector<int> &candidates,
                                std::vector<std::pair<int, int>> *out) {
        const std::set<int> candidateSet(candidates.begin(), candidates.end());
        std::set<int> covered;
        // Disjoint changed subtrees, in slot (== namespace) order.
        std::vector<int> roots(candidateSet.begin(), candidateSet.end());
        int coveredRoot = -1;
        for (int root : roots) {
            if (coveredRoot >= 0 &&
                B.paths[root].HasPrefix(B.paths[coveredRoot])) {
                continue;
            }
            coveredRoot = root;
            for (int j = root + 1; j < N; ++j) {
                if (!B.paths[j].HasPrefix(B.paths[root]))
                    break;
                // Only FirstFramePose slots are propagated to: a plain
                // Xformable a constraint targets is only seeded and revised. It
                // still takes a slot, so the scan passes over it rather than
                // stopping at it.
                if (B.slotKind[size_t(j)] !=
                    RigExecBakedSlotKind::FirstFramePose) {
                    continue;
                }
                if (candidateSet.count(j) || !covered.insert(j).second) {
                    continue;
                }
                // The closest revised ancestor is a pure namespace climb, so it
                // rides propParent and not parent: an xform-derived ancestor
                // can be the revised one.
                int closest = B.propParent[j];
                while (closest >= 0 && !candidateSet.count(closest)) {
                    closest = B.propParent[closest];
                }
                if (closest < 0)
                    continue;
                // An independently solved joint is an absolute posed
                // override: propagation cannot pass through it.
                bool blocked = false;
                for (int p = j; p >= 0 && p != closest; p = B.propParent[p]) {
                    if (ownedBySolver[p]) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked)
                    continue;
                out->emplace_back(j, closest);
            }
        }
    };

    for (const RigExecBakedWalkEntry &entry : walk) {
        RigExecBakedProgramImpl::WalkStep st;
        st.solverBatch = entry.solverBatch;
        if (entry.solverBatch) {
            st.level = entry.level;
            std::vector<int> candidates;
            for (size_t k = 0; k < entry.batchSolvers.size(); ++k) {
                const int slot =
                    bakeSolver(entry.batchSolvers[k], entry.solverJoints[k],
                               k < entry.solverSnapshotJoints.size()
                                   ? entry.solverSnapshotJoints[k]
                                   : std::vector<char>(),
                               k < entry.solverLiveRestJoints.size()
                                   ? entry.solverLiveRestJoints[k]
                                   : SdfPathVector());
                st.batchSolvers.push_back(slot);
                for (const auto &[providerSlot, element] :
                     B.solvers[slot].outputs) {
                    candidates.push_back(providerSlot);
                }
            }
            buildPropagation(candidates, &st.propagate);
        } else {
            st.index = bakeConstraint(entry.constraint);
            const RigExecBakedProgramImpl::Constraint &baked =
                B.constraints[size_t(st.index)];
            // A geometry-domain constraint revises NO transform: it measures a
            // delta and hands it to a Matrix revision. It still reads its
            // target's frame as the input it solves from, which is the commit's
            // targetRead and not a candidate. A SingleChainIK revises its whole
            // chain atomically, so the propagation is built over all of it at
            // once; buildPropagation collapses a nested chain to its root.
            if (baked.target >= 0 && baked.pointsTarget.IsEmpty()) {
                buildPropagation(baked.singleChainIk
                                     ? baked.targetSlots
                                     : std::vector<int>{baked.target},
                                 &st.propagate);
            }
        }
        B.walkSteps.push_back(std::move(st));
    }

    // The solvers no batch runs. A solver that binds no joint and that no mover
    // names on rigExec:driverFrames is not required, so the walk never reaches
    // it. It is still an aggregate solver whose frames publish as guides,
    // computed against the walk's FINAL provider frames. Baked here, in the
    // dependency order Build resolved, and run as
    // ordinary Solve steps after the walk.
    for (const SdfPath &solverPath : ctx->guideOnlySolvers) {
        B.guideSolvers.push_back(bakeSolver(solverPath, {}, {}, {}));
    }
    // Aggregate identities are bound after all descriptors exist; authored
    // discovery order does not decide whether an upstream solver is visible.
    for (auto &solver : B.solvers)
        if (solver.type == SchemaTypeName::RigExecBlendPointFrames) {
            const auto prim = B.stage->GetPrimAtPath(solver.path);
            const auto source = [&](const TfToken &name) {
                SdfPathVector paths;
                if (const auto rel = prim.GetRelationship(name))
                    rel.GetTargets(&paths);
                const auto found =
                    paths.empty() ? B.solverIndex.end()
                                  : B.solverIndex.find(paths[0].GetPrimPath());
                return found == B.solverIndex.end() ? -1 : found->second;
            };
            solver.inA = source(_tokens->rigExecInputA);
            solver.inB = source(_tokens->rigExecInputB);
        }
    {
        // A guide-only blend's op node is added after its inputs'.
        std::vector<char> placed(B.solvers.size(), 0);
        std::vector<int> ordered;
        const std::function<void(int)> place = [&](int si) {
            if (si < 0 || placed[size_t(si)])
                return;
            placed[size_t(si)] = 1;
            if (std::find(B.guideSolvers.begin(), B.guideSolvers.end(), si) ==
                B.guideSolvers.end())
                return;
            place(B.solvers[size_t(si)].inA);
            place(B.solvers[size_t(si)].inB);
            ordered.push_back(si);
        };
        for (const int si : B.guideSolvers)
            place(si);
        B.guideSolvers = std::move(ordered);
    }
    {
        // 0 not yet counted, 1 counting, 2 counted.
        std::vector<char> counted(B.solvers.size(), 0);
        const std::function<size_t(int)> frames = [&](int i) -> size_t {
            RigExecBakedProgramImpl::Solver &s = B.solvers[size_t(i)];
            if (counted[size_t(i)])
                return s.frameCount;
            counted[size_t(i)] = 1;
            if (s.kind != RigExecSolverKind::BlendPointFrames) {
                s.frameCount = RigExecSolverOwnFrameCount(s);
            } else if (s.inA >= 0 && s.inB >= 0) {
                if (s.blendRotationRejected)
                    refuse("frame blend rigExec:rotationBlend is not "
                           "shortestArc",
                           s.path);
                const size_t a = frames(s.inA), b = frames(s.inB);
                if (a != b)
                    refuse("frame blend inputs publish " + std::to_string(a) +
                               " and " + std::to_string(b) +
                               " frames; a blend needs equal counts",
                           s.path);
                s.frameCount = a;
            } else if (s.inA >= 0 || s.inB >= 0) {
                s.frameCount = frames(s.inA >= 0 ? s.inA : s.inB);
            }
            counted[size_t(i)] = 2;
            return s.frameCount;
        };
        for (size_t i = 0; i < B.solvers.size(); ++i)
            frames(int(i));
    }
    // Retain only supported solver-input relationships as semantic
    // prerequisites. Generic relationships and authored stack order are not
    // imported into this list. All solver identities exist before this census.
    const TfToken knownPorts[] = {
        _tokens->rigExecControls,    _tokens->rigExecStartFrame,
        _tokens->rigExecRootControl, _tokens->rigExecEffectorControl,
        _tokens->rigExecPoleControl, _tokens->rigExecSpace,
        _tokens->rigExecInputA,      _tokens->rigExecInputB,
        _tokens->rigExecStart,       _tokens->rigExecEnd,
        _tokens->rigExecDriverCurve, _tokens->rigExecEndFrame,
        _tokens->rigExecTwistFrames, _tokens->rigExecMidControl,
        _tokens->rigExecEndControl};
    const auto portToken = [&](const std::string &port) {
        for (const TfToken &known : knownPorts) {
            if (known == port) {
                return known;
            }
        }
        return TfToken(port);
    };
    for (auto &solver : B.solvers) {
        for (const auto &port : RigExecSolverRelationshipPorts(solver.kind)) {
            auto targets = compileInputs.Targets(
                solver.path.AppendProperty(portToken(port)));
            // Structural-only Ribbon ports retain every authored target.
            // Numerical input ports select the binder's first target; FK
            // controls retain their ordered list.
            const bool ribbonStructural =
                solver.kind == RigExecSolverKind::Ribbon &&
                port != SchemaAttrText::rigExecDriverCurve;
            if (port != SchemaAttrText::rigExecControls && !ribbonStructural &&
                targets.size() > 1)
                targets.resize(1);
            for (const auto &target : targets) {
                const auto found = B.solverIndex.find(target.GetPrimPath());
                if (found != B.solverIndex.end())
                    solver.relationshipRequirements.emplace_back(port,
                                                                 found->second);
            }
        }
        std::sort(solver.relationshipRequirements.begin(),
                  solver.relationshipRequirements.end());
        solver.relationshipRequirements.erase(
            std::unique(solver.relationshipRequirements.begin(),
                        solver.relationshipRequirements.end()),
            solver.relationshipRequirements.end());
    }
    B.aggregates.resize(B.solvers.size());
}

// Translate authored pose checkpoints to typed operation reads and writes.
// The common compiler assigns execution order after all domains are present.

namespace {

/// Appends a step of \p kind about \p object and returns it.
RigExecBakedStep &
AddStep(RigExecBakedProgramImpl *program, RigExecBakedStepKind kind, int object,
        int part = -1)
{
    RigExecBakedStep step;
    step.kind = kind;
    step.object = object;
    step.part = part;
    step.maxDiagnostics = 0;
    program->steps.push_back(std::move(step));
    return program->steps.back();
}

/// An op node step of \p family keeping the identity of \p kind.
RigExecBakedStep &
AddOriginStep(RigExecBakedProgramImpl *program, RigExecBakedOpFamily family,
              RigExecBakedStepKind kind, int object, int part)
{
    RigExecBakedOpOrigin origin;
    origin.family = family;
    origin.kind = kind;
    origin.object = int32_t(object);
    origin.part = int32_t(part);
    RigExecBakedStep &step = AddStep(program, RigExecBakedStepKind::OpNode,
                                     int(program->opNodeOrigins.size()));
    step.maxDiagnostics = 1;
    program->opNodeOrigins.push_back(origin);
    return step;
}

/// Op node \p part of the \p nodes solver \p si runs as; the last keeps
/// the Solve step's identity.
RigExecBakedStep &
AddSolverStep(RigExecBakedProgramImpl *program, int si, size_t part,
              size_t nodes)
{
    return AddOriginStep(program, RigExecBakedOpFamily::Solver,
                         part + 1 == nodes ? RigExecBakedStepKind::Solve
                                           : RigExecBakedStepKind::OpNode,
                         si, part + 1 == nodes ? -1 : int(part));
}

/// Binds every pose read and write of the walk to a VERSION of its slot.
///
/// One sweep in program order over the same writes the edge sweep sees,
/// keeping the entry that holds each slot's current version. A read is bound
/// to the entry standing when the reader runs; a write gets an entry of its
/// own, which nothing else in the program writes. Slot i's first version is
/// entry i -- the compose writes there, an xform-derived slot's seed sits
/// there -- so a rig with no commits allocates nothing and the compose body
/// needs no table at all.
///
/// The carry entries are what a write site that does not write means: it
/// copies the version standing at ITS point into its own storage, so every
/// version a later reader can name is well formed however the run went.
void
BindPoseVersions(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t n = B.paths.size();
    // How many versions each slot ends up with, counted before any is handed
    // out. The LAST version of a slot is what the matrices, the phased-read
    // records and the publication all read -- 252 matrix steps and ~850 keys
    // on a biped -- so those entries are laid out DENSELY, one per slot at
    // [n, 2n), and only the versions in between go into the arena past them.
    // Without that the frame's most-walked reads scatter over the whole
    // version table and the publication alone costs a third more.
    std::vector<uint32_t> finWrites(n, 0), baseWrites(n, 0);
    RigExecBakedPlanProviderRefreshes(&B);
    for (const auto &refresh : B.providerRefreshes) {
        ++baseWrites[size_t(refresh.slot)];
        ++finWrites[size_t(refresh.slot)];
        for (const auto &carry : refresh.carries) {
            ++baseWrites[size_t(carry.slot)];
            ++finWrites[size_t(carry.slot)];
        }
    }
    for (const RigExecBakedCommit &commit : B.commits) {
        for (const int slot : commit.slots) {
            ++finWrites[size_t(slot)];
            if (commit.solverOutput) {
                ++baseWrites[size_t(slot)];
            }
        }
        for (const auto &[descendant, closest] : commit.propagate) {
            ++finWrites[size_t(descendant)];
            if (commit.solverOutput) {
                ++baseWrites[size_t(descendant)];
            }
        }
    }
    // Slot -> the entry holding its current version, as the sweep stands,
    // and how many of its versions are still to come.
    std::vector<uint32_t> liveFin(n), liveBase(n);
    for (size_t i = 0; i < n; ++i) {
        liveFin[i] = uint32_t(i);
        liveBase[i] = uint32_t(i);
    }
    uint32_t finNext = uint32_t(2 * n), baseNext = uint32_t(2 * n);
    // The next entry for one slot: its dense last-version entry when this is
    // the last write of it, an arena entry otherwise.
    const auto take = [n](std::vector<uint32_t> *remaining, uint32_t *next,
                          size_t slot) {
        return --(*remaining)[slot] == 0 ? uint32_t(n + slot) : (*next)++;
    };
    const auto readFin = [&liveFin, n](int slot) {
        return slot >= 0 && size_t(slot) < n ? liveFin[size_t(slot)] : 0u;
    };
    const auto readBase = [&liveBase, n](int slot) {
        return slot >= 0 && size_t(slot) < n ? liveBase[size_t(slot)] : 0u;
    };
    RigExecProviderContextIndex refreshContextIndex;
    // Refresh binding only clones operations and selects frame versions; it
    // does not append external inputs. Index those fixed rows once, lazily.
    std::vector<std::vector<size_t>> refreshExternalInputs;
    if (!B.providerRefreshes.empty()) {
        refreshExternalInputs.resize(n);
        for (size_t k = 0; k < B.providerProgram.externalInputs.size(); ++k) {
            const auto &input = B.providerProgram.externalInputs[k];
            if (input.computation != SchemaComputationText::computePointFrame)
                continue;
            const auto found = B.index.find(input.owner);
            if (found != B.index.end())
                refreshExternalInputs[size_t(found->second)].push_back(k);
        }
    }
    const auto refreshBefore = [&](size_t checkpoint) {
        for (uint32_t index : B.providerRefreshBefore[checkpoint])
            if (!RigExecBakedBindProviderRefresh(
                    &B, index, &liveBase, &liveFin,
                    [&](bool base, int slot) {
                        return base ? take(&baseWrites, &baseNext, size_t(slot))
                                    : take(&finWrites, &finNext, size_t(slot));
                    },
                    &refreshContextIndex, refreshExternalInputs))
                return false;
        return true;
    };
    // One solver's frame reads, bound to the versions live where it runs.
    // Written once because two callers need it at two different points of
    // the sweep: a batch's solvers read the versions live where their batch
    // begins, and the guide-only solvers -- which run after the whole walk
    // -- read the last version of everything.
    const auto bindSolverReads =
        [&readFin](RigExecBakedProgramImpl::Solver &solver) {
            solver.controlReads.clear();
            for (const int control : solver.controls) {
                solver.controlReads.push_back(readFin(control));
            }
            solver.startRead = readFin(solver.start);
            solver.rootRead = readFin(solver.root);
            solver.midRead = readFin(solver.mid);
            solver.endRead = readFin(solver.end);
            solver.poleRead = readFin(solver.pole);
            solver.spaceRead = readFin(solver.spaceSlot);
            // The live joint rests, bound like the control reads: to the
            // version standing where this batch begins.
            if (solver.hasLiveRest) {
                solver.restReads.assign(solver.restRefs.size(), 0u);
                for (size_t k = 0; k < solver.restRefs.size(); ++k) {
                    if (k < solver.restIsLive.size() && solver.restIsLive[k]) {
                        solver.restReads[k] = readFin(solver.restRefs[k].first);
                    }
                }
            }
        };

    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
        if (!refreshBefore(w))
            return;
        const RigExecBakedProgramImpl::WalkStep &walk = B.walkSteps[w];
        RigExecBakedCommit &commit = B.commits[w];
        // The producers first: a batch's solvers and a constraint's own
        // inputs read the versions live where the batch begins, which is
        // where the commit's own reads are taken too.
        if (walk.solverBatch) {
            for (const int si : walk.batchSolvers) {
                bindSolverReads(B.solvers[size_t(si)]);
            }
        } else {
            const RigExecBakedProgramImpl::Constraint &constraint =
                B.constraints[size_t(walk.index)];
            commit.sourceReads.clear();
            for (const int source : constraint.sources) {
                commit.sourceReads.push_back(readFin(source));
            }
            commit.worldUpRead = readFin(constraint.worldUpObject);
            // The frame the constraint solves FROM, whichever domain it
            // writes. For a transform-domain constraint this is the same
            // version its one candidate reads; a geometry-domain one has no
            // candidate at all and this is its only read of the target.
            commit.targetRead = readFin(constraint.target);
            commit.targetReads.clear();
            for (const int slot : constraint.targetSlots) {
                commit.targetReads.push_back(readFin(slot));
            }
            // A native source rides the deepest ancestor the walk has moved
            // BY THIS POINT, so both halves of every candidate ancestor --
            // its base and its final -- are read at the versions live here.
            const auto ancestorReads =
                [&](int native,
                    std::vector<RigExecBakedCommit::AncestorRead> *out) {
                    out->clear();
                    if (native < 0) {
                        return;
                    }
                    for (const int slot :
                         B.nativeSources[size_t(native)].ancestorSlots) {
                        out->push_back({slot, readFin(slot), readBase(slot)});
                    }
                };
            commit.sourceAncestors.resize(constraint.sources.size());
            for (size_t k = 0; k < constraint.sourceNatives.size(); ++k) {
                ancestorReads(constraint.sourceNatives[k],
                              &commit.sourceAncestors[k]);
            }
            ancestorReads(constraint.worldUpNative, &commit.worldUpAncestors);
            commit.effectorRead = readFin(constraint.effector);
            ancestorReads(constraint.effectorNative, &commit.effectorAncestors);
            commit.poleReads.clear();
            commit.poleAncestors.assign(constraint.poleObjects.size(), {});
            for (size_t k = 0; k < constraint.poleObjects.size(); ++k) {
                commit.poleReads.push_back(readFin(constraint.poleObjects[k]));
                ancestorReads(constraint.poleObjectNatives[k],
                              &commit.poleAncestors[k]);
            }
        }
        commit.slotReads.clear();
        for (const int slot : commit.slots) {
            commit.slotReads.push_back(readFin(slot));
        }
        commit.descendantReads.clear();
        commit.closestReads.clear();
        for (const auto &[descendant, closest] : commit.propagate) {
            commit.descendantReads.push_back(readFin(descendant));
            commit.closestReads.push_back(readFin(closest));
        }

        // Then the write-back, in the order it happens: the candidates in
        // slot order, then the descendants in propagation order. A slot that
        // is both gets two versions, and the second carries the first.
        commit.slotWrites.assign(commit.slots.size(), 0);
        commit.slotCarry.assign(commit.slots.size(), 0);
        commit.slotBaseWrites.assign(
            commit.solverOutput ? commit.slots.size() : 0, 0);
        commit.slotBaseCarry.assign(
            commit.solverOutput ? commit.slots.size() : 0, 0);
        for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
            const size_t slot = size_t(commit.slots[pos]);
            commit.slotCarry[pos] = liveFin[slot];
            commit.slotWrites[pos] = take(&finWrites, &finNext, slot);
            liveFin[slot] = commit.slotWrites[pos];
            if (commit.solverOutput) {
                commit.slotBaseCarry[pos] = liveBase[slot];
                commit.slotBaseWrites[pos] = take(&baseWrites, &baseNext, slot);
                liveBase[slot] = commit.slotBaseWrites[pos];
            }
        }
        commit.descendantWrites.assign(commit.propagate.size(), 0);
        commit.descendantCarry.assign(commit.propagate.size(), 0);
        commit.descendantBaseWrites.assign(
            commit.solverOutput ? commit.propagate.size() : 0, 0);
        commit.descendantBaseCarry.assign(
            commit.solverOutput ? commit.propagate.size() : 0, 0);
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            const size_t slot = size_t(commit.propagate[k].first);
            commit.descendantCarry[k] = liveFin[slot];
            commit.descendantWrites[k] = take(&finWrites, &finNext, slot);
            liveFin[slot] = commit.descendantWrites[k];
            if (commit.solverOutput) {
                commit.descendantBaseCarry[k] = liveBase[slot];
                commit.descendantBaseWrites[k] =
                    take(&baseWrites, &baseNext, slot);
                liveBase[slot] = commit.descendantBaseWrites[k];
            }
        }
    }

    if (!refreshBefore(B.walkSteps.size()))
        return;
    // All writes are indexed before ordinary data-flow inputs are bound.
    // A dependency can name a later authored writer; scheduling never changes
    // the positional carry versions or the producer selected here.
    std::set<uint32_t> refreshedBase, refreshedFin;
    for (const auto &refresh : B.providerRefreshes) {
        refreshedBase.insert(refresh.baseWrite);
        refreshedFin.insert(refresh.finWrite);
        for (const auto &carry : refresh.carries) {
            refreshedBase.insert(carry.baseWrite);
            refreshedFin.insert(carry.finWrite);
        }
    }
    const auto selected = [&](const SdfPath &reader, int slot,
                              uint32_t positional, bool base = false) {
        if (slot < 0 || size_t(slot) >= n)
            return positional;
        if ((base ? refreshedBase : refreshedFin).count(positional))
            return positional;
        const auto dependencies = B.poseDependencyPaths.find(reader);
        if (dependencies == B.poseDependencyPaths.end())
            return positional;
        uint32_t value = positional;
        for (size_t w = 0; w < B.walkSteps.size(); ++w) {
            const auto &walk = B.walkSteps[w];
            const auto &commit = B.commits[w];
            bool reads = false;
            if (walk.solverBatch)
                for (const int si : walk.batchSolvers)
                    reads = reads || dependencies->second.count(
                                         B.solvers[size_t(si)].path);
            else
                reads = dependencies->second.count(
                    B.constraints[size_t(walk.index)].path);
            if (!reads || (base && !commit.solverOutput))
                continue;
            const auto &writes =
                base ? commit.slotBaseWrites : commit.slotWrites;
            for (size_t k = 0; k < commit.slots.size(); ++k)
                if (commit.slots[k] == slot && k < writes.size())
                    value = writes[k];
            const auto &descendants =
                base ? commit.descendantBaseWrites : commit.descendantWrites;
            for (size_t k = 0; k < commit.propagate.size(); ++k)
                if (commit.propagate[k].first == slot && k < descendants.size())
                    value = descendants[k];
        }
        return value;
    };
    for (auto &input : B.providerFrameInputs)
        input.version =
            selected(input.reader, input.slot, input.version, input.base);
    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
        const auto &walk = B.walkSteps[w];
        auto &commit = B.commits[w];
        if (walk.solverBatch) {
            for (const int si : walk.batchSolvers) {
                auto &solver = B.solvers[size_t(si)];
                for (size_t k = 0; k < solver.controls.size(); ++k)
                    solver.controlReads[k] =
                        selected(solver.path, solver.controls[k],
                                 solver.controlReads[k]);
                solver.startRead =
                    selected(solver.path, solver.start, solver.startRead);
                solver.rootRead =
                    selected(solver.path, solver.root, solver.rootRead);
                solver.midRead =
                    selected(solver.path, solver.mid, solver.midRead);
                solver.endRead =
                    selected(solver.path, solver.end, solver.endRead);
                solver.poleRead =
                    selected(solver.path, solver.pole, solver.poleRead);
                solver.spaceRead =
                    selected(solver.path, solver.spaceSlot, solver.spaceRead);
            }
            continue;
        }
        const auto &constraint = B.constraints[size_t(walk.index)];
        for (size_t k = 0; k < constraint.sources.size(); ++k)
            commit.sourceReads[k] = selected(
                constraint.path, constraint.sources[k], commit.sourceReads[k]);
        commit.worldUpRead = selected(constraint.path, constraint.worldUpObject,
                                      commit.worldUpRead);
        commit.effectorRead =
            selected(constraint.path, constraint.effector, commit.effectorRead);
        for (size_t k = 0; k < constraint.poleObjects.size(); ++k)
            commit.poleReads[k] =
                selected(constraint.path, constraint.poleObjects[k],
                         commit.poleReads[k]);
        const auto ancestors = [&](auto &reads) {
            for (auto &read : reads) {
                read.fin = selected(constraint.path, read.slot, read.fin);
                read.base =
                    selected(constraint.path, read.slot, read.base, true);
            }
        };
        for (auto &reads : commit.sourceAncestors)
            ancestors(reads);
        ancestors(commit.worldUpAncestors);
        ancestors(commit.effectorAncestors);
        for (auto &reads : commit.poleAncestors)
            ancestors(reads);
    }

    // The guide-only solvers read every provider's FINAL frame, which is what
    // liveFin holds now that the sweep is over.
    for (const int si : B.guideSolvers) {
        bindSolverReads(B.solvers[size_t(si)]);
    }

    B.finLast.assign(liveFin.begin(), liveFin.end());
    B.baseLast.assign(liveBase.begin(), liveBase.end());
    B.fin.resize(finNext);
    B.base.resize(baseNext);
}

/// Builds `frameRecords` and every AtPrim reader's lists of them, before any
/// commit step exists: records are named by (provider, writer) and need no
/// version. A record is a target a constraint's exit records
/// (`snapshotTargets`) or an output a solver's commit records
/// (`outputSnapshots`) -- the compile's named pairs -- kept only when some
/// reader's list holds it. A list is the original AtPrim lookup
/// rule stated statically: the provider's records whose writer is the
/// phase's prim or under it, newest first in walk order.
void
EnumerateFrameRecords(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    std::vector<RigExecBakedFrameRecord> all;
    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
        const RigExecBakedProgramImpl::WalkStep &walk = B.walkSteps[w];
        if (walk.solverBatch) {
            // The walk records every named joint of every solver in the
            // batch once the batch commits. `position` is bound with the
            // version, once the commit's slot table exists.
            for (const int si : walk.batchSolvers) {
                const RigExecBakedProgramImpl::Solver &solver =
                    B.solvers[size_t(si)];
                for (size_t k = 0; k < solver.outputs.size(); ++k) {
                    if (k < solver.outputSnapshots.size() &&
                        solver.outputSnapshots[k] &&
                        solver.outputs[k].first >= 0) {
                        RigExecBakedFrameRecord record;
                        record.slot = solver.outputs[k].first;
                        record.commit = int(w);
                        record.mover = solver.path;
                        all.push_back(std::move(record));
                    }
                }
            }
            continue;
        }
        const RigExecBakedProgramImpl::Constraint &constraint =
            B.constraints[size_t(walk.index)];
        for (size_t k = 0; k < constraint.targetSlots.size(); ++k) {
            if (k < constraint.snapshotTargets.size() &&
                constraint.snapshotTargets[k] &&
                constraint.targetSlots[k] >= 0) {
                RigExecBakedFrameRecord record;
                record.slot = constraint.targetSlots[k];
                record.commit = int(w);
                record.target = int(k);
                record.mover = constraint.path;
                all.push_back(std::move(record));
            }
        }
    }
    std::vector<char> used(all.size(), 0);
    const auto listFor = [&all, &used](int slot, const SdfPath &prim,
                                       std::vector<int> *out) {
        out->clear();
        if (slot < 0) {
            return;
        }
        for (size_t r = all.size(); r-- > 0;) {
            if (all[r].slot == slot &&
                (all[r].mover == prim || all[r].mover.HasPrefix(prim))) {
                out->push_back(int(r));
                used[r] = 1;
            }
        }
    };
    const auto bindReader =
        [&listFor](RigExecBakedProgramImpl::GeomRevision *revision) {
            revision->transformRecords.clear();
            revision->influenceRecords.clear();
            const RigExecReadPhase &phase = revision->binding.transformPhase;
            if (phase.kind != RigExecReadPhaseKind::AtPrim) {
                return;
            }
            listFor(revision->transformSlot, phase.prim,
                    &revision->transformRecords);
            // The fold substitutes an influence entry only where the binding
            // names one, as the walk's fold does.
            revision->influenceRecords.resize(revision->influenceSlots.size());
            for (size_t k = 0; k < revision->influenceSlots.size() &&
                               k < revision->binding.influences.size();
                 ++k) {
                listFor(revision->influenceSlots[k], phase.prim,
                        &revision->influenceRecords[k]);
            }
        };
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            bindReader(&revision);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            bindReader(&derived.revision);
        }
    }
    std::vector<int> renumbered(all.size(), -1);
    B.frameRecords.clear();
    for (size_t r = 0; r < all.size(); ++r) {
        if (used[r]) {
            renumbered[r] = int(B.frameRecords.size());
            B.frameRecords.push_back(std::move(all[r]));
        }
    }
    const auto renumber = [&renumbered](std::vector<int> *records) {
        for (int &record : *records) {
            record = renumbered[size_t(record)];
        }
    };
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            renumber(&revision.transformRecords);
            for (std::vector<int> &records : revision.influenceRecords) {
                renumber(&records);
            }
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            renumber(&derived.revision.transformRecords);
            for (std::vector<int> &records :
                 derived.revision.influenceRecords) {
                renumber(&records);
            }
        }
    }
    B.frameMatrix.assign(B.frameRecords.size(), GfMatrix4d(1.0));
    B.frameMatrixValid.assign(B.frameRecords.size(), 0);
}

/// Binds each record to the `fin` entry its writer left the provider in,
/// once BindPoseVersions has handed the entries out: the commit's LAST
/// version of the slot where it declares one -- whether it revised it or
/// carried the one it found -- which is the frame the walk records after the
/// commit's propagation; else the version it read, which is where a
/// geometry-domain constraint leaves its target. A solver's record also
/// takes the slot's position in the commit, whose `present` byte gates it.
void
BindFrameRecordVersions(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedFrameRecord &record : B.frameRecords) {
        const RigExecBakedCommit &commit = B.commits[size_t(record.commit)];
        const auto found = std::lower_bound(commit.slots.begin(),
                                            commit.slots.end(), record.slot);
        const bool candidate =
            found != commit.slots.end() && *found == record.slot;
        if (commit.solverOutput) {
            // Every solver output is a slot of its commit (the sizing loop
            // adds each one), so a miss is a broken table.
            TF_VERIFY(candidate);
            record.position =
                candidate ? int(found - commit.slots.begin()) : -1;
        }
        if (!candidate) {
            record.version = record.target >= 0 && size_t(record.target) <
                                                       commit.targetReads.size()
                                 ? commit.targetReads[size_t(record.target)]
                                 : 0;
            continue;
        }
        record.version =
            commit.slotWrites[size_t(found - commit.slots.begin())];
        // The propagation pairs exclude every candidate (buildPropagation).
        // Binds to the slot's last commit version.
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            if (commit.propagate[k].first == record.slot) {
                record.version = commit.descendantWrites[k];
            }
        }
    }
}

/// What a switched slot's compose reads besides its own avars and its
/// namespace parent's anchor (which the group lists in parentSlots): the
/// PosedM slots read at their last version, and the slots whose avars an
/// earlier version is recomposed from.
void
_SwitchVersionReads(const RigExecBakedProgramImpl::SpaceSwitch &sw,
                    std::vector<int> *posed, std::vector<int> *avars)
{
    const auto add =
        [&](const RigExecBakedProgramImpl::SpaceSwitch::FrameVersion &read) {
            if (read.anchor >= 0) {
                posed->push_back(read.anchor);
            }
            avars->insert(avars->end(), read.recompose.begin(),
                          read.recompose.end());
        };
    for (size_t k = 0; k < sw.sourceSlots.size(); ++k) {
        if (sw.sourceSlots[k] >= 0 && k < sw.sourceReads.size()) {
            add(sw.sourceReads[k]);
        }
    }
    if (sw.spaceSlot >= 0) {
        add(sw.spaceRead);
    }
    avars->insert(avars->end(), sw.parentRead.recompose.begin(),
                  sw.parentRead.recompose.end());
}

/// The PosedM slot a slot's compose inherits from: its namespace parent, or
/// for a switched slot the anchor of the parent version it reads.
int
_ComposeParentRead(const RigExecBakedProgramImpl &B, int slot)
{
    const int switchIndex =
        B.spaceSwitchBySlot.empty() ? -1 : B.spaceSwitchBySlot[size_t(slot)];
    return switchIndex >= 0
               ? B.spaceSwitches[size_t(switchIndex)].parentRead.anchor
               : B.parent[size_t(slot)];
}

} // namespace

bool
RigExecBakedBindSpaceSwitchVersions(RigExecBakedProgramImpl *program,
                                    std::string *error)
{
    auto &B = *program;
    using FrameVersion = RigExecBakedProgramImpl::SpaceSwitch::FrameVersion;
    RigExecPathText pathText;
    std::vector<uint8_t> composed(B.paths.size(), 0);
    for (size_t slot = 0; slot < B.paths.size(); ++slot)
        composed[slot] =
            B.slotKind[slot] == RigExecBakedSlotKind::FirstFramePose;
    std::vector<int> targets, carrySlots;
    std::vector<std::vector<int>> sources;
    for (const auto &sw : B.spaceSwitches) {
        targets.push_back(sw.slot);
        sources.push_back(sw.sourceSlots);
        carrySlots.push_back(sw.spaceSlot);
    }
    std::vector<RigExecSpaceParentContext> contexts, carryContexts;
    if (!RigExecDeriveSpaceSwitchParentContexts(B.parent, composed, targets,
                                                sources, &contexts, &carrySlots,
                                                &carryContexts)) {
        if (error)
            *error = "space switch checkpoint has an invalid provider identity";
        return false;
    }
    B.switchFrameContexts.clear();
    for (size_t index = 0; index < B.spaceSwitches.size(); ++index) {
        auto &sw = B.spaceSwitches[index];
        const auto bind = [&](const RigExecSpaceParentContext &selected,
                              const char *site) {
            FrameVersion read;
            if (selected.recompose.empty())
                read.anchor = selected.anchor;
            else {
                RigExecBakedProgramImpl::SpaceCheckpoint checkpoint;
                checkpoint.key =
                    "switchCheckpoint:" + pathText(B.paths[size_t(sw.slot)]) +
                    ":" + site;
                checkpoint.anchor = selected.anchor;
                checkpoint.recompose = selected.recompose;
                read.context = int(B.switchFrameContexts.size());
                B.switchFrameContexts.push_back(std::move(checkpoint));
            }
            return read;
        };
        sw.parentRead = bind(contexts[index], "parent");
        sw.spaceRead = bind(carryContexts[index], "carry");
        sw.sourceReads.assign(sw.sourceSlots.size(), FrameVersion());
        for (size_t k = 0; k < sw.sourceSlots.size(); ++k)
            sw.sourceReads[k].anchor = sw.sourceSlots[k];
    }
    B.switchFrames.assign(B.switchFrameContexts.size(), GfMatrix4d(1.0));
    return true;
}

void
RigExecBakedBuildPoseSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const int N = int(B.paths.size());
    for (size_t index = 0; index < B.switchFrameContexts.size(); ++index) {
        const auto &context = B.switchFrameContexts[index];
        auto &step = AddOriginStep(&B, RigExecBakedOpFamily::SpaceCheckpoint,
                                   RigExecBakedStepKind::SpaceCheckpoint,
                                   int(index), -1);
        step.label = context.key;
        if (context.anchor >= 0)
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PosedM,
                                                 context.anchor));
        for (int slot : context.recompose) {
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::Avars, slot));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::Ladder, slot));
        }
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::SwitchFrame, int(index)));
    }

    // A provider is a semantic operation. Packing occurs after the common
    // graph has bound every connected and phased input to its producer.
    std::vector<size_t> emission;
    for (int slot = 0; slot < N; ++slot) {
        // Ordinary Xform targets are sampled frame seeds, not composed
        // providers. A skipped body must not claim their source versions.
        if (B.slotKind[size_t(slot)] != RigExecBakedSlotKind::FirstFramePose)
            continue;
        RigExecBakedComposeGroup group;
        group.begin = slot;
        group.end = slot + 1;
        const int parent = _ComposeParentRead(B, slot);
        if (parent >= 0)
            group.parentSlots.push_back(parent);
        emission.push_back(B.composeGroups.size());
        B.composeGroups.push_back(std::move(group));
    }
    for (const size_t g : emission) {
        const RigExecBakedComposeGroup &group = B.composeGroups[g];
        RigExecBakedStep &step =
            AddStep(&B, RigExecBakedStepKind::ComposeSubtree, int(g));
        step.reads.push_back(RigExecBakedRange(RigExecBakedSlotDomain::Avars,
                                               group.begin, group.end));
        for (const int parent : group.parentSlots) {
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PosedM, parent));
        }
        // A switched slot reads its spaces instead of its namespace parent,
        // so those frames are reads of this step too -- that is what orders
        // a pole vector's group after its IK handle's. A source at -1 is
        // world; an own-source read remains a real dependency for SCC policy.
        for (int slot = group.begin; slot < group.end; ++slot) {
            const int switchIndex = B.spaceSwitchBySlot.empty()
                                        ? -1
                                        : B.spaceSwitchBySlot[size_t(slot)];
            if (switchIndex < 0)
                continue;
            const RigExecBakedProgramImpl::SpaceSwitch &sw =
                B.spaceSwitches[size_t(switchIndex)];
            // A space match that refuses a frame says so.
            ++step.maxDiagnostics;
            const auto checkpointRead = [&](const auto &read) {
                if (read.context >= 0)
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::SwitchFrame, read.context));
            };
            checkpointRead(sw.parentRead);
            checkpointRead(sw.spaceRead);
            for (const auto &read : sw.sourceReads)
                checkpointRead(read);
            std::vector<int> reads, recomposed;
            _SwitchVersionReads(sw, &reads, &recomposed);
            // An earlier version recomposed here reads those slots' avars,
            // declared so that moving one dirties this step as well as the
            // group that composes the slot's last version
            // (RigExecBakedCones::avarVersionSteps).
            std::sort(recomposed.begin(), recomposed.end());
            recomposed.erase(std::unique(recomposed.begin(), recomposed.end()),
                             recomposed.end());
            for (const int at : recomposed) {
                if (at < group.begin || at >= group.end) {
                    step.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::Avars, at, at + 1));
                }
            }
            for (const int source : reads)
                if (source >= 0)
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PosedM, source));
            // The index itself is declared by
            // RigExecBakedDeclareInputDependencies.
        }
        for (int slot = group.begin; slot < group.end; ++slot) {
            const int index = (size_t(slot) < B.autoClavicleBySlot.size()
                                   ? B.autoClavicleBySlot[size_t(slot)]
                                   : -1);
            if (index < 0)
                continue;
            const auto &ac = B.autoClavicles[size_t(index)];
            for (const auto &read : ac.frames) {
                if (read.computation == SchemaComputationText::computeRestFrame)
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::Rest, read.slot));
                else if (read.computation ==
                         SchemaComputationText::computeDefaultFrame)
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::Ladder, read.slot));
                for (int child : read.recompose) {
                    step.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::Avars, child));
                    step.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::Ladder, child));
                }
            }
        }
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PoseBase, group.begin, group.end));
        step.writes.push_back(RigExecBakedRange(RigExecBakedSlotDomain::PoseFin,
                                                group.begin, group.end));
        step.writes.push_back(RigExecBakedRange(RigExecBakedSlotDomain::PosedM,
                                                group.begin, group.end));
    }

    B.commits.resize(B.walkSteps.size());
    EnumerateFrameRecords(&B);
    // One FrameMatrix step per record, right after its commit's last step.
    // It reads the provider's frame -- an edge from the latest writer of the
    // slot so far, which is this commit or the producer of the version it
    // read -- and the commit's table (a constraint's exit flags, a solver
    // batch's `present` bytes), which its first step writes (CommitTable).
    size_t nextRecord = 0;
    const auto addFrameRecordSteps = [&B, &nextRecord](size_t w) {
        for (; nextRecord < B.frameRecords.size() &&
               size_t(B.frameRecords[nextRecord].commit) == w;
             ++nextRecord) {
            const RigExecBakedFrameRecord &record = B.frameRecords[nextRecord];
            RigExecBakedStep &step = AddOriginStep(
                &B, RigExecBakedOpFamily::FrameMatrix,
                RigExecBakedStepKind::FrameMatrix, int(nextRecord), -1);
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, record.slot));
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::CommitTable, record.commit));
            step.writes.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::FrameMatrix, int(nextRecord)));
        }
    };
    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
        const RigExecBakedProgramImpl::WalkStep &walk = B.walkSteps[w];
        RigExecBakedCommit &commit = B.commits[w];
        commit.solverOutput = walk.solverBatch;
        commit.propagate = walk.propagate;
        // The candidate table is dense and in SLOT order, because both
        // halves of the commit -- the usability sweep and the write-back --
        // walk it in slot order. Where each producer lands is decided here,
        // so the merge is an indexed store and "last writer wins on a
        // duplicate slot" survives as the order the stores happen in.
        if (walk.solverBatch) {
            // Each solver owns its authored commit and distinct pose versions.
            for (const int si : walk.batchSolvers) {
                for (const auto &[slot, element] :
                     B.solvers[size_t(si)].outputs) {
                    commit.slots.push_back(slot);
                }
            }
        } else {
            const RigExecBakedProgramImpl::Constraint &constraint =
                B.constraints[size_t(walk.index)];
            commit.moverPath = constraint.path;
            commit.moverPathText = constraint.pathText;
            if (constraint.target >= 0 && constraint.pointsTarget.IsEmpty()) {
                if (constraint.singleChainIk) {
                    // The one multi-target built-in: every chain joint is a
                    // candidate, and the commit is atomic over all of them.
                    commit.slots.insert(commit.slots.end(),
                                        constraint.targetSlots.begin(),
                                        constraint.targetSlots.end());
                } else {
                    commit.slots.push_back(constraint.target);
                }
            }
            commit.sources.resize(constraint.sources.size());
            commit.ikChain.resize(constraint.targetSlots.size());
            commit.ikRest.resize(constraint.targetSlots.size());
            commit.ikPrepared.reserve(constraint.targetSlots.size());
        }
        std::sort(commit.slots.begin(), commit.slots.end());
        commit.slots.erase(
            std::unique(commit.slots.begin(), commit.slots.end()),
            commit.slots.end());
        const auto positionOf = [&commit](int slot) {
            const auto found = std::lower_bound(commit.slots.begin(),
                                                commit.slots.end(), slot);
            return found != commit.slots.end() && *found == slot
                       ? int(found - commit.slots.begin())
                       : -1;
        };
        commit.present.assign(commit.slots.size(), 0);
        commit.frames.resize(commit.slots.size());
        commit.deltas.assign(commit.slots.size(), GfMatrix4d(1.0));
        commit.deltaOk.assign(commit.slots.size(), 0);
        commit.staged.resize(commit.propagate.size());
        commit.outcome.assign(commit.propagate.size(), 0);
        commit.closestPos.reserve(commit.propagate.size());
        for (const auto &[descendant, closest] : commit.propagate) {
            commit.closestPos.push_back(positionOf(closest));
        }

        if (walk.solverBatch) {
            for (const int si : walk.batchSolvers) {
                RigExecBakedProgramImpl::Solver &solver = B.solvers[size_t(si)];
                solver.outFrames.resize(solver.outputs.size());
                solver.outPresent.assign(solver.outputs.size(), 0);
                solver.outPosition.clear();
                for (const auto &[slot, element] : solver.outputs) {
                    solver.outPosition.push_back(positionOf(slot));
                }
                const size_t nodes = RigExecBakedSolverNodeCount(solver);
                for (size_t part = 0; part < nodes; ++part) {
                    RigExecBakedStep &step = AddSolverStep(&B, si, part, nodes);
                    // A refused or failed solve reports why.
                    step.maxDiagnostics = 1;
                    for (const int control : solver.controls) {
                        if (control >= 0) {
                            step.reads.push_back(RigExecBakedOne(
                                RigExecBakedSlotDomain::PoseFin, control));
                        }
                    }
                    for (const int control :
                         {solver.root, solver.mid, solver.end, solver.pole}) {
                        if (control >= 0) {
                            step.reads.push_back(RigExecBakedOne(
                                RigExecBakedSlotDomain::PoseFin, control));
                        }
                    }
                    // The start provider's posed frame: undeclared, the
                    // scheduler is free to run this solve before the write.
                    if (solver.start >= 0) {
                        step.reads.push_back(RigExecBakedOne(
                            RigExecBakedSlotDomain::PoseFin, solver.start));
                    }
                    // The LIVE joint rests: a solver whose rest reference is the
                    // frame a pose step below it left READS that slot's PoseFin,
                    // and a read a step does not declare is a read the scheduler
                    // is free to run before the write.
                    // Empty on every solver with no step below it.
                    if (solver.hasLiveRest) {
                        for (size_t k = 0; k < solver.restRefs.size(); ++k) {
                            if (k < solver.restIsLive.size() &&
                                solver.restIsLive[k] &&
                                solver.restRefs[k].first >= 0) {
                                step.reads.push_back(RigExecBakedOne(
                                    RigExecBakedSlotDomain::PoseFin,
                                    solver.restRefs[k].first));
                            }
                        }
                    }
                    // The ribbon's driver points, filled by the prologue.
                    // Nothing in the program writes the domain, so the read
                    // raises no edge; it is declared because it is what the cone
                    // follows when the curve moves, and because a step's
                    // declaration is meant to say what the step touches.
                    if (!solver.ribbonPointsPath.IsEmpty()) {
                        step.reads.push_back(RigExecBakedOne(
                            RigExecBakedSlotDomain::SolverPoints, si));
                    }
                    // An input solver's Solve step precedes this one; the
                    // validator refuses a program where it does not.
                    for (const int input : {solver.inA, solver.inB}) {
                        if (input >= 0) {
                            step.reads.push_back(RigExecBakedOne(
                                RigExecBakedSlotDomain::Aggregate, input));
                        }
                    }
                    if (part + 1 < nodes)
                        continue;
                    step.writes.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::Aggregate, si));
                    step.writes.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::Candidates, si));
                }
            }
        }

        // A constraint with no target solves nothing and has no step.
        if (!walk.solverBatch && !RigExecBakedConstraintRunsAsOps(B, w)) {
            addFrameRecordSteps(w);
            continue;
        }
        // A constraint's node leaves its candidates in the commit table; a
        // PoseCommit op node writes them back and carries them onto the
        // descendants. A solver batch's node gathers its solvers' candidates
        // itself and publishes which are present in the table.
        RigExecBakedOpOrigin origin;
        origin.family = RigExecBakedOpFamily::PoseCommit;
        origin.kind = walk.solverBatch ? RigExecBakedStepKind::SolverCommit
                                       : RigExecBakedStepKind::CommitApply;
        origin.object = int32_t(w);
        RigExecBakedStep &commitStep =
            walk.solverBatch
                ? AddStep(&B, RigExecBakedStepKind::OpNode,
                          int(B.opNodeOrigins.size()))
                : AddOriginStep(&B, RigExecBakedOpFamily::Constraint,
                                RigExecBakedStepKind::Constraint, int(w), -1);
        if (walk.solverBatch)
            B.opNodeOrigins.push_back(origin);
        commitStep.maxDiagnostics = RigExecBakedMaxStepDiagnostics;
        if (walk.solverBatch) {
            for (const int si : walk.batchSolvers) {
                commitStep.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::Candidates, si));
            }
        } else {
            const RigExecBakedProgramImpl::Constraint &constraint =
                B.constraints[size_t(walk.index)];
            for (const int source : constraint.sources) {
                if (source >= 0) {
                    commitStep.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseFin, source));
                }
            }
            if (constraint.target >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, constraint.target));
            }
            if (constraint.worldUpObject >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, constraint.worldUpObject));
            }
            // rigExec:space: the carry is read off the compose's own
            // table, exactly as a switched slot reads its space, so moving
            // a master dirties this step.
            if (constraint.spaceSlot >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PosedM, constraint.spaceSlot));
            }
            // Both frames of every provider a native source might ride: the
            // step compares them to decide which ancestor moved, so both are
            // reads whatever this frame's answer turns out to be.
            const auto declareNative = [&B, &commitStep](int native) {
                if (native < 0) {
                    return;
                }
                for (const int slot :
                     B.nativeSources[size_t(native)].ancestorSlots) {
                    commitStep.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
                    commitStep.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseBase, slot));
                }
            };
            for (const int native : constraint.sourceNatives) {
                declareNative(native);
            }
            declareNative(constraint.worldUpNative);
            // A SingleChainIK reads its whole chain, its effector and every
            // pole object.
            for (const int slot : constraint.targetSlots) {
                commitStep.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
            }
            if (constraint.effector >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, constraint.effector));
            }
            // An IK/FK match reads its controls' channels and the inputs its
            // pole's compose left (written by the same step that writes the
            // pole's base frame); its rests are declared with every step's
            // (RigExecBakedRequiredRestReads).
            if (constraint.ikFkMatch) {
                for (size_t k = 5; k < constraint.sources.size(); ++k) {
                    commitStep.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::Avars, constraint.sources[k]));
                }
            }
            declareNative(constraint.effectorNative);
            for (size_t k = 0; k < constraint.poleObjects.size(); ++k) {
                if (constraint.poleObjects[k] >= 0) {
                    commitStep.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,
                                        constraint.poleObjects[k]));
                }
                declareNative(constraint.poleObjectNatives[k]);
            }
        }
        commitStep.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::CommitTable, int(w)));
        // The commit reads its descendants and their closest revised
        // ancestors and writes them back: a read-modify-write of both, which
        // is what the propagation IS.
        const auto declarePropagation = [&B, &commit](RigExecBakedStep *step,
                                                      bool writes) {
            for (const auto &[descendant, closest] : commit.propagate) {
                step->reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, descendant));
                step->reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, closest));
                if (!writes) {
                    continue;
                }
                step->writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, descendant));
                if (commit.solverOutput) {
                    step->writes.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseBase, descendant));
                }
            }
            if (!writes) {
                return;
            }
            for (const int slot : commit.slots) {
                step->writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
                if (commit.solverOutput) {
                    step->writes.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseBase, slot));
                }
            }
        };
        // What the write-back carries is a read too, of a different version
        // than the delta's: wherever it declines to write, the commit copies
        // the version it found.
        const auto declareCarryReads = [&commit](RigExecBakedStep *step) {
            for (const int slot : commit.slots) {
                step->reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
                if (commit.solverOutput) {
                    step->reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseBase, slot));
                }
            }
            if (!commit.solverOutput) {
                return;
            }
            for (const auto &pair : commit.propagate) {
                step->reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseBase, pair.first));
            }
        };
        RigExecBakedStep *finish = &commitStep;
        if (!walk.solverBatch) {
            const int deltaBase = B.constraints[size_t(walk.index)].deltaBase;
            if (deltaBase >= 0) {
                RigExecBakedStep &delta =
                    AddOriginStep(&B, RigExecBakedOpFamily::Constraint,
                                  RigExecBakedStepKind::Constraint, int(w), 0);
                delta.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::CommitTable, int(w)));
                delta.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ConstraintDelta, deltaBase));
            }
            if (commit.slots.empty() && commit.propagate.empty()) {
                addFrameRecordSteps(w);
                continue;
            }
            finish = &AddStep(&B, RigExecBakedStepKind::OpNode,
                              int(B.opNodeOrigins.size()));
            B.opNodeOrigins.push_back(origin);
            finish->maxDiagnostics = RigExecBakedMaxStepDiagnostics;
            finish->reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::CommitTable, int(w)));
        }
        // The delta is measured against each candidate slot's value BEFORE
        // the commit revises it, so that read is real and not implied by the
        // write.
        for (const int slot : commit.slots) {
            finish->reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
        }
        declarePropagation(finish, /* writes = */ true);
        declareCarryReads(finish);
        addFrameRecordSteps(w);
    }
    // Before the matrices, which read the LAST version of a slot: the table
    // that says which entry that is comes out of this sweep.
    BindPoseVersions(&B);
    for (auto &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::ComposeSubtree)
            continue;
        const auto &group = B.composeGroups[size_t(step.object)];
        for (const auto &ac : B.autoClavicles) {
            if (ac.slot < group.begin || ac.slot >= group.end)
                continue;
            // A frame the clavicle reads outside its own subtree is that
            // provider's compose version: the frame its channels give, which
            // the solvers read too, and not a constraint's later revision.
            for (const auto &read : ac.frames)
                if (read.computation.empty() && read.recompose.empty() &&
                    read.slot != ac.slot)
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseFin, uint32_t(read.slot)));
        }
    }

    if (!B.providerRefreshError.empty())
        return;
    for (size_t index = 0; index < B.providerRefreshes.size(); ++index) {
        const auto &refresh = B.providerRefreshes[index];
        auto &step =
            AddOriginStep(&B, RigExecBakedOpFamily::ProviderRefresh,
                          RigExecBakedStepKind::ProviderRefresh, int(index), 0);
        step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,
                                             refresh.baseValue));
        step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,
                                             refresh.currentValue));
        step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,
                                             refresh.baseRead));
        step.reads.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, refresh.finRead));
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,
                                              refresh.baseWrite));
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, refresh.finWrite));
        for (const auto &carry : refresh.carries) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseBase, carry.baseRead));
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseFin, carry.finRead));
            step.writes.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseBase, carry.baseWrite));
            step.writes.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseFin, carry.finWrite));
            for (int slot : carry.blockingSlots) {
                const int raw = B.providerParentRawLeaves[size_t(slot)];
                if (raw >= 0)
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::SpaceLeaf, uint32_t(raw)));
            }
        }
        for (const auto &[commit, pos] : refresh.priorConstraints)
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::CommitTable, commit));
    }
    BindFrameRecordVersions(&B);

    // Runs after the walk (reads final versions); writes no candidate, bumps no
    // counter. Not gated on the guide toggle: the epilogue consults it.
    for (const int si : B.guideSolvers) {
        RigExecBakedProgramImpl::Solver &solver = B.solvers[size_t(si)];
        const size_t nodes = RigExecBakedSolverNodeCount(solver);
        for (size_t part = 0; part < nodes; ++part) {
            RigExecBakedStep &step = AddSolverStep(&B, si, part, nodes);
            for (const int control : solver.controls) {
                if (control >= 0) {
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseFin, control));
                }
            }
            for (const int control :
                 {solver.root, solver.mid, solver.end, solver.pole}) {
                if (control >= 0) {
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseFin, control));
                }
            }
            if (solver.start >= 0) {
                step.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, solver.start));
            }
            if (!solver.ribbonPointsPath.IsEmpty()) {
                step.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::SolverPoints, si));
            }
            for (const int input : {solver.inA, solver.inB}) {
                if (input >= 0) {
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::Aggregate, input));
                }
            }
            if (part + 1 < nodes)
                continue;
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::Aggregate, si));
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::Candidates, si));
        }
    }

    // Final for every published joint, and the reads
    // RigExecBakedForEachMatrixRead enumerates for every geometry revision.
    // A ProviderMatrix step never gives the generation back --
    // RigExecPointsToMatrix leaves the identity and says so -- so the one
    // bail of this phase stays where it is, in the joint publication.
    B.needFinal.assign(size_t(N), 0);
    B.needBase.assign(size_t(N), 0);
    B.finalMatrix.assign(size_t(N), GfMatrix4d(1.0));
    B.baseMatrix.assign(size_t(N), GfMatrix4d(1.0));
    for (const int slot : B.jointSlots) {
        if (slot >= 0) {
            B.needFinal[size_t(slot)] = 1;
        }
    }
    const auto needForRevision =
        [&B](const RigExecBakedProgramImpl::GeomRevision &revision) {
            RigExecBakedForEachMatrixRead(
                revision, [&B](RigExecBakedSlotDomain domain, int slot) {
                    std::vector<char> &table =
                        domain == RigExecBakedSlotDomain::FinalMatrix
                            ? B.needFinal
                            : B.needBase;
                    table[size_t(slot)] = 1;
                });
        };
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            needForRevision(revision);
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            needForRevision(derived.revision);
        }
    }
    for (int slot = 0; slot < N; ++slot) {
        if (B.needFinal[size_t(slot)]) {
            RigExecBakedStep &step =
                AddOriginStep(&B, RigExecBakedOpFamily::ProviderMatrix,
                              RigExecBakedStepKind::ProviderMatrix, slot, 1);
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseBase, slot));
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::FinalMatrix, slot));
        }
        if (B.needBase[size_t(slot)]) {
            RigExecBakedStep &step =
                AddOriginStep(&B, RigExecBakedOpFamily::ProviderMatrix,
                              RigExecBakedStepKind::ProviderMatrix, slot, 0);
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseBase, slot));
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::BaseMatrix, slot));
        }
    }

    // After the matrices: an interpolator reads the FINAL pose, every
    // constraint included, and writes what the geometry chains consume.
    // One step per interpolator, reading the last version of its driver's slot
    // and of the parent's, writing its own range of weights.
    for (size_t k = 0; k < B.poseInterpolators.size(); ++k) {
        const RigExecBakedProgramImpl::PoseInterpolator &interpolator =
            B.poseInterpolators[k];
        RigExecBakedStep &step =
            AddOriginStep(&B, RigExecBakedOpFamily::PoseInterpolator,
                          RigExecBakedStepKind::PoseInterpolator, int(k), -1);
        // A numeric driver reads dials and no frame at all, so it names no
        // slot to read: pushing one would read slot -1.
        if (interpolator.driverSlot >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseFin, interpolator.driverSlot));
        }
        if (interpolator.parentSlot >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseFin, interpolator.parentSlot));
        }
        for (size_t e = 0; e < interpolator.extraDriverSlots.size(); ++e) {
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,
                                interpolator.extraDriverSlots[e]));
            if (interpolator.extraParentSlots[e] >= 0) {
                step.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,
                                    interpolator.extraParentSlots[e]));
            }
        }
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PoseWeight, interpolator.weightBegin,
            interpolator.weightEnd));
    }

    // The body bindings are the operation's value identities. Provider slots
    // name only the initial compose values; every authored commit owns its
    // assigned SSA outputs, independent of the common graph's eventual order.
    using Domain = RigExecBakedSlotDomain;
    using Kind = RigExecBakedStepKind;
    for (auto &step : B.steps) {
        const RigExecBakedStepIdentity id = RigExecBakedIdentityOf(B, step);
        const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
        const bool commitNode =
            origin && origin->family == RigExecBakedOpFamily::PoseCommit;
        const bool constraintStep = id.kind == Kind::Constraint && id.part < 0;
        const int solverIndex = RigExecBakedSolverOf(B, step);
        if (solverIndex < 0 && id.kind != Kind::FrameMatrix &&
            id.kind != Kind::ProviderMatrix &&
            id.kind != Kind::PoseInterpolator && !constraintStep && !commitNode)
            continue;
        const auto poseRange = [](const RigExecBakedSlotRange &range) {
            return range.domain == Domain::PoseFin ||
                   range.domain == Domain::PoseBase;
        };
        step.reads.erase(
            std::remove_if(step.reads.begin(), step.reads.end(), poseRange),
            step.reads.end());
        step.writes.erase(
            std::remove_if(step.writes.begin(), step.writes.end(), poseRange),
            step.writes.end());
        const auto read = [&](Domain domain, uint32_t value) {
            step.reads.push_back(RigExecBakedOne(domain, value));
        };
        const auto readAll = [&](Domain domain, const auto &values) {
            for (const auto value : values)
                read(domain, value);
        };
        const auto writeAll = [&](Domain domain, const auto &values) {
            for (const auto value : values)
                step.writes.push_back(RigExecBakedOne(domain, value));
        };
        if (solverIndex >= 0) {
            const auto &solver = B.solvers[size_t(solverIndex)];
            readAll(Domain::PoseFin, solver.controlReads);
            for (const auto binding :
                 {std::make_pair(solver.start, solver.startRead),
                  std::make_pair(solver.root, solver.rootRead),
                  std::make_pair(solver.mid, solver.midRead),
                  std::make_pair(solver.end, solver.endRead),
                  std::make_pair(solver.pole, solver.poleRead),
                  std::make_pair(solver.spaceSlot,
                                 uint32_t(solver.spaceRead))}) {
                if (binding.first >= 0)
                    read(Domain::PoseFin, binding.second);
            }
            for (size_t k = 0; k < solver.restReads.size(); ++k)
                if (k < solver.restIsLive.size() && solver.restIsLive[k])
                    read(Domain::PoseFin, solver.restReads[k]);
        } else if (id.kind == Kind::FrameMatrix) {
            read(Domain::PoseFin, B.frameRecords[size_t(id.object)].version);
        } else if (id.kind == Kind::ProviderMatrix) {
            const size_t slot = size_t(id.object);
            if (id.part)
                read(Domain::PoseFin, B.finLast[slot]);
            read(Domain::PoseBase, B.baseLast[slot]);
        } else if (id.kind == Kind::PoseInterpolator) {
            const auto &interp = B.poseInterpolators[size_t(id.object)];
            if (interp.driverSlot >= 0)
                read(Domain::PoseFin, B.finLast[size_t(interp.driverSlot)]);
            if (interp.parentSlot >= 0)
                read(Domain::PoseFin, B.finLast[size_t(interp.parentSlot)]);
            for (size_t e = 0; e < interp.extraDriverSlots.size(); ++e) {
                read(Domain::PoseFin,
                     B.finLast[size_t(interp.extraDriverSlots[e])]);
                if (interp.extraParentSlots[e] >= 0)
                    read(Domain::PoseFin,
                         B.finLast[size_t(interp.extraParentSlots[e])]);
            }
        } else if (constraintStep || commitNode) {
            const auto &commit =
                B.commits[size_t(commitNode ? origin->object : id.object)];
            if (constraintStep) {
                const auto &c =
                    B.constraints[size_t(B.walkSteps[size_t(id.object)].index)];
                for (size_t k = 0; k < c.sources.size(); ++k)
                    if (c.sources[k] >= 0)
                        read(Domain::PoseFin, commit.sourceReads[k]);
                if (c.target >= 0)
                    read(Domain::PoseFin, commit.targetRead);
                readAll(Domain::PoseFin, commit.targetReads);
                if (c.worldUpObject >= 0)
                    read(Domain::PoseFin, commit.worldUpRead);
                if (c.effector >= 0)
                    read(Domain::PoseFin, commit.effectorRead);
                for (size_t k = 0; k < c.poleObjects.size(); ++k)
                    if (c.poleObjects[k] >= 0)
                        read(Domain::PoseFin, commit.poleReads[k]);
                const auto ancestors = [&](const auto &values) {
                    for (const auto &value : values) {
                        read(Domain::PoseFin, value.fin);
                        read(Domain::PoseBase, value.base);
                    }
                };
                for (const auto &values : commit.sourceAncestors)
                    ancestors(values);
                for (const auto &values : commit.poleAncestors)
                    ancestors(values);
                ancestors(commit.worldUpAncestors);
                ancestors(commit.effectorAncestors);
                // The inputs an IK/FK match re-runs its pole's space from
                // are left by the compose that writes the pole's base.
                if (c.ikFkMatch && c.sources.size() == 10)
                    read(Domain::PoseBase, B.baseLast[size_t(c.sources[6])]);
            }
            if (commitNode) {
                readAll(Domain::PoseFin, commit.slotReads);
                for (size_t k = 0; k < commit.propagate.size(); ++k) {
                    read(Domain::PoseFin, commit.descendantReads[k]);
                    read(Domain::PoseFin, commit.closestReads[k]);
                }
                writeAll(Domain::PoseFin, commit.slotWrites);
                writeAll(Domain::PoseFin, commit.descendantWrites);
                writeAll(Domain::PoseBase, commit.slotBaseWrites);
                writeAll(Domain::PoseBase, commit.descendantBaseWrites);
                for (auto value : commit.slotCarry)
                    read(Domain::PoseFin, value);
                for (auto value : commit.slotBaseCarry)
                    read(Domain::PoseBase, value);
                const auto carries = [&](Domain domain, const auto &values,
                                         const auto &candidateWrites) {
                    for (size_t k = 0; k < values.size(); ++k) {
                        bool internal = false;
                        for (size_t pos = 0; pos < commit.slots.size() &&
                                             pos < candidateWrites.size();
                             ++pos)
                            internal =
                                internal || (commit.propagate[k].first ==
                                                 commit.slots[pos] &&
                                             values[k] == candidateWrites[pos]);
                        if (!internal)
                            read(domain, values[k]);
                    }
                };
                carries(Domain::PoseFin, commit.descendantCarry,
                        commit.slotWrites);
                carries(Domain::PoseBase, commit.descendantBaseCarry,
                        commit.slotBaseWrites);
            }
        }
    }
}

// One frame, pose half.
// Everything below is a STEP BODY or one half of the serial prologue or
// epilogue. A body touches no pose, takes no lock, opens no profile scope and
// writes only the slots its step declared; whatever it has to say it says
// into its own step.

namespace {

/// Records \p input against \p step: what a frame can move it with.
///
/// The rule itself is RigExecBakedNoteInput in bakedProgramImpl.h, because
/// the weight half declares its inputs the same way and two spellings of
/// "what can move this" is a step the cone cannot dirty.
template <class T>
void
NoteInput(const RigExecBakedInput<T> &input, RigExecBakedDependencySink *sink)
{
    RigExecBakedNoteInput(input, sink);
}

/// Every per-frame input one solver's Solve step reads.
void
NoteSolverInputs(const RigExecBakedProgramImpl::Solver &solver,
                 RigExecBakedDependencySink *sink)
{
    // The rest description is one of them. A rest that varies with time
    // moves it whenever the time moves, and a drag on a rest channel of the
    // same chain moves it where no time did -- the two halves the schedule
    // asks about separately.
    sink->step->varyingInputs = sink->step->varyingInputs || solver.restsVary;
    sink->step->overrideInputs.insert(sink->step->overrideInputs.end(),
                                      solver.restOverrides.begin(),
                                      solver.restOverrides.end());
    NoteInput(solver.bend, sink);
    NoteInput(solver.upperOffset, sink);
    NoteInput(solver.lowerOffset, sink);
    NoteInput(solver.stretch, sink);
    NoteInput(solver.softness, sink);
    NoteInput(solver.pin, sink);
    NoteInput(solver.upperScale, sink);
    NoteInput(solver.lowerScale, sink);
    NoteInput(solver.softDistance, sink);
    NoteInput(solver.limbTwist, sink);
    NoteInput(solver.blendWeight, sink);
    NoteInput(solver.preserveVolume, sink);
    NoteInput(solver.midFollowWeight, sink);
    NoteInput(solver.roll, sink);
    NoteInput(solver.twist, sink);
    NoteInput(solver.minLengthRatio, sink);
    NoteInput(solver.twistTurns, sink);
    NoteInput(solver.ribbonSampleCount, sink);
    NoteInput(solver.ikSpace, sink);
    for (const auto &input : solver.bone2dDoubles)
        NoteInput(input, sink);
    for (const auto &input : solver.bone2dBools)
        NoteInput(input, sink);
    for (const auto &input : solver.bone2dSelectors)
        NoteInput(input, sink);
    for (const auto &input : solver.jointGroupSelectors)
        NoteInput(input, sink);
    // The spline parameters the bake could not fold, which the solve re-reads
    // as a group rather than one input at a time.
    sink->step->varyingInputs =
        sink->step->varyingInputs || solver.splineParamsVary;
}

/// Every per-frame input one constraint's step reads.
void
NoteConstraintInputs(const RigExecBakedProgramImpl::Constraint &constraint,
                     RigExecBakedDependencySink *sink)
{
    NoteInput(constraint.enabled, sink);
    NoteInput(constraint.defaultWeight, sink);
    NoteInput(constraint.offset, sink);
    NoteInput(constraint.affectX, sink);
    NoteInput(constraint.affectY, sink);
    NoteInput(constraint.affectZ, sink);
    NoteInput(constraint.tX, sink);
    NoteInput(constraint.tY, sink);
    NoteInput(constraint.tZ, sink);
    NoteInput(constraint.rX, sink);
    NoteInput(constraint.rY, sink);
    NoteInput(constraint.rZ, sink);
    NoteInput(constraint.sX, sink);
    NoteInput(constraint.sY, sink);
    NoteInput(constraint.sZ, sink);
    NoteInput(constraint.aimVector, sink);
    NoteInput(constraint.upVector, sink);
    NoteInput(constraint.rotationOffset, sink);
    NoteInput(constraint.worldUpVector, sink);
    // The two SingleChainIK-only inputs. Bound only in RotatePlane mode, so
    // in every other mode these are the default-constructed inputs and note
    // nothing -- which is the same answer as not listing them, and a good
    // deal harder to forget.
    NoteInput(constraint.poleVector, sink);
    NoteInput(constraint.twistDegrees, sink);
    NoteInput(constraint.stretch, sink);
    // An IK/FK match's scalars, each in the one precision it is bound in.
    if (constraint.ikFkMatch) {
        for (const auto &scalar : constraint.matchScalars) {
            if (scalar.isFloat)
                NoteInput(scalar.narrow, sink);
            else
                NoteInput(scalar.wide, sink);
        }
    }
    // A harmonics constraint's scalars, likewise.
    if (constraint.harmonics) {
        for (const auto &scalar : constraint.harmonicsScalars) {
            if (scalar.isFloat)
                NoteInput(scalar.narrow, sink);
            else
                NoteInput(scalar.wide, sink);
        }
    }
    // The authored source-weight, offset and pole-weight tables are NOT
    // noted here: they are not inputs the step reads at all. The prologue
    // reads them off the stage as compiled rig state (again whenever the stage edit
    // serial, the program stamp or a read that varies with the time moved;
    // ConstraintArrays) and compares them by value, and
    // `constraintArrayClusters` is what dirties this step when one moved
    // (RigExecBakedComputeClosure) -- the source mechanism, not the
    // varying-input one.
}

// One binding-to-step association shared by declarations and cone classification.
void
NoteStepInputs(const RigExecBakedProgramImpl &B,
               RigExecBakedDependencySink *sink)
{
    RigExecBakedStep &step = *sink->step;
    const RigExecBakedStepIdentity identity = RigExecBakedIdentityOf(B, step);
    if (identity.kind == RigExecBakedStepKind::WeightPacket) {
        if (identity.object >= 0 &&
            size_t(identity.object) < B.weightObjects.size())
            RigExecBakedNoteWeightInputs(
                B.weightObjects[size_t(identity.object)], sink);
        return;
    }
    if (const int refresh = RigExecBakedProviderRefreshOf(B, step);
        refresh >= 0) {
        for (const auto &carry : B.providerRefreshes[size_t(refresh)].carries)
            for (int slot : carry.blockingSlots)
                NoteInput(B.ladders[size_t(slot)].parentSpace, sink);
        return;
    }
    if (identity.kind == RigExecBakedStepKind::AvarInputs) {
        const auto note = [&](const auto &bindings,
                              const std::vector<uint32_t> &begin) {
            const auto range =
                RigExecBakedAvarBindingRange(begin, identity.object);
            for (size_t i = range.first; i < range.second; ++i)
                NoteInput(bindings[i].input, sink);
        };
        note(B.avarBindings, B.avarBindingBegin);
        note(B.avarConstantBindings, B.avarConstantBindingBegin);
        return;
    }
    if (identity.kind == RigExecBakedStepKind::Constraint) {
        if (identity.object < 0 ||
            size_t(identity.object) >= B.walkSteps.size())
            return;
        // A commit step and its walk entry are the same index, and a
        // constraint entry names the constraint it commits.
        const RigExecBakedProgramImpl::WalkStep &walk =
            B.walkSteps[size_t(identity.object)];
        if (!walk.solverBatch && walk.index >= 0 &&
            size_t(walk.index) < B.constraints.size()) {
            const auto &constraint = B.constraints[size_t(walk.index)];
            NoteConstraintInputs(constraint, sink);
            if (constraint.arrays >= 0)
                sink->step->reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::ConstraintInputs,
                                    uint32_t(constraint.arrays)));
        }
        return;
    }
    if (identity.kind == RigExecBakedStepKind::PoseInterpolator) {
        if (identity.object < 0 ||
            size_t(identity.object) >= B.poseInterpolators.size())
            return;
        const RigExecBakedProgramImpl::PoseInterpolator &interpolator =
            B.poseInterpolators[size_t(identity.object)];
        NoteInput(interpolator.enabled, sink);
        for (size_t k = 0; k < interpolator.valueInputs.size(); ++k) {
            if (k < interpolator.valueIsFloat.size() &&
                interpolator.valueIsFloat[k]) {
                NoteInput(interpolator.floatValueInputs[k], sink);
            } else {
                NoteInput(interpolator.valueInputs[k], sink);
            }
        }
        if (interpolator.mlSolve) {
            NoteInput(interpolator.mask, sink);
        }
        return;
    }
    const int solverIndex = RigExecBakedSolverOf(B, step);
    switch (solverIndex >= 0 ? RigExecBakedStepKind::Solve : step.kind) {
    case RigExecBakedStepKind::Solve:
        if (size_t(solverIndex) >= B.solvers.size())
            break;
        NoteSolverInputs(B.solvers[size_t(solverIndex)], sink);
        break;
    case RigExecBakedStepKind::ComposeSubtree: {
        if (step.object < 0 || size_t(step.object) >= B.composeGroups.size())
            break;
        // A switched slot's active index, so that keying a space dirties
        // the compose group that reads it and only that one.
        const RigExecBakedComposeGroup &group =
            B.composeGroups[size_t(step.object)];
        for (int slot = group.begin; slot < group.end; ++slot) {
            const int ac = (size_t(slot) < B.autoClavicleBySlot.size()
                                ? B.autoClavicleBySlot[size_t(slot)]
                                : -1);
            if (ac >= 0)
                for (const auto &read : B.autoClavicles[size_t(ac)].scalars) {
                    if (read.isFloat)
                        NoteInput(read.narrow, sink);
                    else
                        NoteInput(read.wide, sink);
                }
            const int switchIndex =
                slot < 0 || size_t(slot) >= B.spaceSwitchBySlot.size()
                    ? -1
                    : B.spaceSwitchBySlot[size_t(slot)];
            if (switchIndex >= 0 &&
                size_t(switchIndex) < B.spaceSwitches.size()) {
                const auto &sw = B.spaceSwitches[size_t(switchIndex)];
                if (sw.tokenIndex)
                    NoteInput(sw.activeTokenInput, sink);
                else
                    NoteInput(sw.activeInput, sink);
            }
        }
        break;
    }
    default:
        break;
    }
}

} // namespace

std::vector<char>
RigExecBakedResolvedReaders(const RigExecBakedProgramImpl &B)
{
    std::vector<char> resolved(B.steps.size(), 0);
    for (size_t i = 0; i < B.steps.size(); ++i) {
        if (B.steps[i].isHead)
            continue;
        RigExecBakedStep scratch;
        scratch.kind = B.steps[i].kind;
        scratch.object = B.steps[i].object;
        RigExecBakedDependencySink sink{&scratch};
        NoteStepInputs(B, &sink);
        resolved[i] = sink.resolvedReads;
    }
    return resolved;
}

void
RigExecBakedDeclareInputDependencies(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedStep &step : B.steps) {
        if (step.isHead) {
            RigExecBakedDeclareWeightField(&B, &step);
            continue;
        }
        step.varyingInputs = false;
        step.overrideInputs.clear();
        step.readerWalks.clear();
        step.reads.erase(std::remove_if(step.reads.begin(), step.reads.end(),
                                        [](const auto &r) {
                                            return RigExecBakedIsHeadDomain(
                                                r.domain);
                                        }),
                         step.reads.end());
        RigExecBakedDependencySink sink{&step};
        NoteStepInputs(B, &sink);
        if (const int f = RigExecBakedWeightFieldOf(B, step); f >= 0) {
            RigExecBakedDeclareWeightField(&B, &step);
            const auto &field = B.weightFields[size_t(f)];
            for (const auto &read : field.scalarReads)
                for (uint32_t leaf : read.leaves)
                    step.varyingInputs =
                        step.varyingInputs || B.headLeaves[leaf].varying;
        }
        if (const RigExecBakedStepIdentity id = RigExecBakedIdentityOf(B, step);
            id.kind == RigExecBakedStepKind::Constraint) {
            const auto &walk = B.walkSteps[size_t(id.object)];
            if (!walk.solverBatch && walk.index >= 0) {
                const auto &constraint = B.constraints[size_t(walk.index)];
                if (constraint.weightField >= 0)
                    step.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::WeightField,
                                        constraint.weightField));
            }
        }
        std::sort(step.overrideInputs.begin(), step.overrideInputs.end());
        step.overrideInputs.erase(
            std::unique(step.overrideInputs.begin(), step.overrideInputs.end()),
            step.overrideInputs.end());
    }
    // The property versions those inputs' walks, and the path leaves'
    // walks, can read; then the rests and ladders the bodies index.
    RigExecBakedDeclareHeadReads(&B);
    RigExecBakedDeclareRestReads(&B);
}

namespace {

// Bit for bit: a NaN that stays NaN has not moved, and -0 is not 0.
template <class T>
bool
_LeafSame(const T &a, const T &b)
{
    if constexpr (std::is_trivially_copyable_v<T>) {
        return std::memcmp(&a, &b, sizeof(T)) == 0;
    } else {
        return a == b;
    }
}

} // namespace

void
RigExecBakedNumberLeaves(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    B.leaves = RigExecBakedLeafPools();
    B.leafRefs.clear();
    B.sourceBackedPaths.clear();
    B.leafOfOverride.assign(B.overridden.size(), -1);
    frozenDetail::_ForEachPatchableInput(B, [&B](auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        using Stored = typename RigExecBakedLeafTraits<T>::Stored;
        if (input.sourceBacked && input.head)
            B.sourceBackedPaths.insert(input.head.GetPath());
        RigExecBakedLeafPool<T> &pool = B.leaves.Of<T>();
        const uint32_t id = uint32_t(B.leafRefs.size());
        input.leaf = int(pool.value.size());
        pool.value.push_back(Stored(input.constant));
        pool.changed.push_back(0);
        pool.mustSample.push_back(0);
        pool.id.push_back(id);
        B.leafRefs.push_back(
            {RigExecBakedLeafTraits<T>::type, uint32_t(input.leaf)});
        if (input.overrideIndex >= 0 &&
            size_t(input.overrideIndex) < B.leafOfOverride.size()) {
            B.leafOfOverride[size_t(input.overrideIndex)] = int(id);
        }
    });
    // RigExecBakedRecordBind files every hop of a registered binding's walk
    // (its head alone when the walk is empty) under the binding's number,
    // so the leaf's paths are exactly those entries.
    B.leafByPath.clear();
    for (const auto &[path, indices] : B.overridableInputs) {
        std::vector<uint32_t> ids;
        for (const int index : indices) {
            if (index >= 0 && size_t(index) < B.leafOfOverride.size() &&
                B.leafOfOverride[size_t(index)] >= 0) {
                ids.push_back(uint32_t(B.leafOfOverride[size_t(index)]));
            }
        }
        if (!ids.empty()) {
            B.leafByPath.emplace_hint(B.leafByPath.end(), path, std::move(ids));
        }
    }
    // The path leaves after them, in program order: chain revisions and
    // derived targets chain by chain, then weight objects, then the skin
    // layouts. Each is filed under every path its read can reach, so a value
    // edit there or on its routed prim marks it (ApplyValueEdits).
    B.pathLeafRefs.clear();
    const auto number = [&B](RigExecBakedPathLeaves &leaves,
                             RigExecBakedPathLeafRef ref) {
        leaves.exactVersions.assign(leaves.decl.keys.size(), -1);
        leaves.exactRecordIndices.assign(leaves.decl.keys.size(), -1);
        leaves.exactValueTypes.assign(leaves.decl.keys.size(), -1);
        for (size_t k = 0; k < leaves.decl.keys.size(); ++k) {
            const auto &key = leaves.decl.keys[k];
            using F = RigExecRevisionLeafFlavour;
            if (ref.owner != RigExecBakedPathLeafOwner::WeightField &&
                key.time == RigExecRevisionLeafTime::AtTime &&
                key.flavour != F::Raw) {
                for (size_t r = 0; r < B.propertyRecords.size(); ++r) {
                    const auto &record = B.propertyRecords[r];
                    if (record.consumer != key.path)
                        continue;
                    leaves.exactVersions[k] = int(record.id);
                    leaves.exactRecordIndices[k] = int(r);
                    leaves.exactValueTypes[k] =
                        int(B.propertyChains[record.chain].arm);
                    break;
                }
                if (leaves.exactVersions[k] < 0)
                    for (const auto &chain : B.propertyChains)
                        if (chain.target == key.path) {
                            leaves.exactVersions[k] =
                                int(chain.versionBase + chain.revisions.size());
                            leaves.exactValueTypes[k] = int(chain.arm);
                            break;
                        }
            }
            ref.key = uint32_t(k);
            const uint32_t id =
                uint32_t(B.leafRefs.size() + B.pathLeafRefs.size());
            B.pathLeafRefs.push_back(ref);
            if (k < leaves.hops.size()) {
                for (const SdfPath &hop : leaves.hops[k]) {
                    B.leafByPath[hop].push_back(id);
                }
            }
        }
    };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            number(chain.revisions[r].leaves,
                   {RigExecBakedPathLeafOwner::Revision, uint32_t(c),
                    uint32_t(r), 0});
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            number(chain.derived[d].revision.leaves,
                   {RigExecBakedPathLeafOwner::Derived, uint32_t(c),
                    uint32_t(d), 0});
        }
    }
    for (size_t w = 0; w < B.weightObjects.size(); ++w) {
        number(B.weightObjects[w].pointLeaves,
               {RigExecBakedPathLeafOwner::Weight, uint32_t(w), 0, 0});
        number(B.weightObjects[w].fieldLeaves,
               {RigExecBakedPathLeafOwner::WeightField, uint32_t(w), 0, 0});
    }
    // The SkinTopology ops' layout leaves, chain by chain.
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            number(chain.revisions[r].layoutLeaves,
                   {RigExecBakedPathLeafOwner::RevisionLayout, uint32_t(c),
                    uint32_t(r), 0});
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            number(chain.derived[d].revision.layoutLeaves,
                   {RigExecBakedPathLeafOwner::DerivedLayout, uint32_t(c),
                    uint32_t(d), 0});
        }
    }
    B.providerLeafEditSerial.assign(B.providerLeaves.decl.keys.size(), 0);
    number(B.providerLeaves, {RigExecBakedPathLeafOwner::Provider, 0, 0, 0});
    const auto declareExact = [&](const RigExecBakedPathLeaves &leaves,
                                  RigExecBakedStep *step,
                                  const RigExecBakedProgramImpl::GeomRevision
                                      *revision = nullptr) {
        // Every declared source opinion participates in the readiness-time
        // memo, including sites without a produced-value reader walk.
        // Numbering precedes this binding so IDs reference the final census.
        for (size_t i = 0; i < B.pathLeafRefs.size(); ++i)
            if (RigExecBakedPathLeavesOf(B, B.pathLeafRefs[i]) == &leaves &&
                (!revision || RigExecBakedGeometryReadsLeaf(
                                  *revision, *step, B.pathLeafRefs[i].key))) {
                const uint32_t id = uint32_t(B.leafRefs.size() + i);
                if (std::find(step->bindingLeaves.begin(),
                              step->bindingLeaves.end(),
                              id) == step->bindingLeaves.end())
                    step->bindingLeaves.push_back(id);
            }
        for (size_t k = 0; k < leaves.exactVersions.size(); ++k)
            if (leaves.exactVersions[k] >= 0 &&
                (!revision ||
                 RigExecBakedGeometryReadsLeaf(*revision, *step, k)))
                step->reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PropertyResult,
                                    uint32_t(leaves.exactVersions[k])));
    };
    const auto directBindings = [&](RigExecBakedStep *step, const auto &visit) {
        visit([&](const auto &input) {
            using T = std::decay_t<decltype(input.constant)>;
            if (input.leaf < 0)
                return;
            const uint32_t id = B.leaves.Of<T>().id[size_t(input.leaf)];
            if (std::find(step->bindingLeaves.begin(),
                          step->bindingLeaves.end(),
                          id) == step->bindingLeaves.end())
                step->bindingLeaves.push_back(id);
            if (input.walk >= 0) {
                const auto binding = std::make_pair(
                    RigExecBakedLeafTraits<T>::type, uint32_t(input.leaf));
                if (std::find(step->walkedBindingLeaves.begin(),
                              step->walkedBindingLeaves.end(),
                              binding) == step->walkedBindingLeaves.end())
                    step->walkedBindingLeaves.push_back(binding);
            }
        });
    };
    for (auto &step : B.steps) {
        // Direct bindings are numbered only now. Their current values must
        // reach both source candidates and the effective numerical memo.
        if (const int si = RigExecBakedSolverOf(B, step); si >= 0) {
            directBindings(&step, [&](const auto &fn) {
                frozenDetail::_VisitSolverInputs(B.solvers[size_t(si)], fn);
            });
        } else if (RigExecBakedIdentityOf(B, step).kind ==
                   RigExecBakedStepKind::Constraint) {
            const auto &walk =
                B.walkSteps[size_t(RigExecBakedIdentityOf(B, step).object)];
            if (!walk.solverBatch && walk.index >= 0)
                directBindings(&step, [&](const auto &fn) {
                    frozenDetail::_VisitConstraintInputs(
                        B.constraints[size_t(walk.index)], fn);
                });
        } else if (RigExecBakedIdentityOf(B, step).kind ==
                   RigExecBakedStepKind::WeightPacket) {
            directBindings(&step, [&](const auto &fn) {
                frozenDetail::_VisitWeightInputs(
                    B.weightObjects[size_t(
                        RigExecBakedIdentityOf(B, step).object)],
                    fn);
            });
        } else if (RigExecBakedIdentityOf(B, step).kind ==
                   RigExecBakedStepKind::PoseInterpolator) {
            directBindings(&step, [&](const auto &fn) {
                frozenDetail::_VisitInterpolatorInputs(
                    B.poseInterpolators[size_t(
                        RigExecBakedIdentityOf(B, step).object)],
                    fn);
            });
        } else if (const int refresh = RigExecBakedProviderRefreshOf(B, step);
                   refresh >= 0) {
            directBindings(&step, [&](const auto &fn) {
                for (const auto &carry :
                     B.providerRefreshes[size_t(refresh)].carries)
                    for (int slot : carry.blockingSlots)
                        fn(B.ladders[size_t(slot)].parentSpace);
            });
            for (const auto &carry :
                 B.providerRefreshes[size_t(refresh)].carries)
                for (int slot : carry.blockingSlots) {
                    const int raw = B.providerParentRawLeaves[size_t(slot)];
                    if (raw < 0)
                        continue;
                    for (size_t i = 0; i < B.pathLeafRefs.size(); ++i) {
                        const auto &ref = B.pathLeafRefs[i];
                        if (ref.owner == RigExecBakedPathLeafOwner::Provider &&
                            ref.key == uint32_t(raw)) {
                            const uint32_t id = uint32_t(B.leafRefs.size() + i);
                            if (std::find(step.bindingLeaves.begin(),
                                          step.bindingLeaves.end(),
                                          id) == step.bindingLeaves.end())
                                step.bindingLeaves.push_back(id);
                            break;
                        }
                    }
                }
        } else if (step.kind == RigExecBakedStepKind::ComposeSubtree) {
            const auto &group = B.composeGroups[size_t(step.object)];
            for (int slot = group.begin; slot < group.end; ++slot) {
                const int sw = B.spaceSwitchBySlot[size_t(slot)];
                if (sw >= 0)
                    directBindings(&step, [&](const auto &fn) {
                        frozenDetail::_VisitSpaceSwitchInputs(
                            B.spaceSwitches[size_t(sw)], fn);
                    });
                const int ac = (size_t(slot) < B.autoClavicleBySlot.size()
                                    ? B.autoClavicleBySlot[size_t(slot)]
                                    : -1);
                if (ac >= 0)
                    directBindings(&step, [&](const auto &fn) {
                        frozenDetail::_VisitAutoClavicleInputs(
                            B.autoClavicles[size_t(ac)], fn);
                    });
            }
        }
        if (step.kind == RigExecBakedStepKind::RevisionStatic ||
            step.kind == RigExecBakedStepKind::RevisionChunk) {
            const auto &at = B.revisionIndex[size_t(step.object)];
            const auto &revision =
                B.chains[size_t(at.first)].revisions[size_t(at.second)];
            declareExact(revision.leaves, &step, &revision);
        } else if (step.kind == RigExecBakedStepKind::OpChainHead &&
                   RigExecBakedIdentityOf(B, step).kind ==
                       RigExecBakedStepKind::RevisionStatic) {
            // An op matrix's head reads its revision's own inputs.
            const auto &at =
                B.revisionIndex[size_t(RigExecBakedIdentityOf(B, step).object)];
            declareExact(
                B.chains[size_t(at.first)].revisions[size_t(at.second)].leaves,
                &step);
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &at = B.derivedIndex[size_t(step.object)];
            declareExact(B.chains[size_t(at.first)]
                             .derived[size_t(at.second)]
                             .revision.leaves,
                         &step);
        } else if (step.kind == RigExecBakedStepKind::SkinTopology) {
            const auto &at = B.revisionIndex[size_t(step.object)];
            declareExact(B.chains[size_t(at.first)]
                             .revisions[size_t(at.second)]
                             .layoutLeaves,
                         &step);
        } else if (RigExecBakedIdentityOf(B, step).kind ==
                   RigExecBakedStepKind::WeightPacket)
            declareExact(
                B.weightObjects[size_t(RigExecBakedIdentityOf(B, step).object)]
                    .pointLeaves,
                &step);
    }
    // Weight-field path leaves are allocated by field capture after the initial
    // census; bind field memo IDs only against this final numbered census.
    for (auto &step : B.steps)
        if (RigExecBakedWeightFieldOf(B, step) >= 0) {
            step.bindingLeaves.clear();
            RigExecBakedDeclareWeightField(&B, &step);
        }
    // The head leaves after those, each under its own path: a head leaf
    // reads one attribute and follows no connection.
    const size_t headBase = B.leafRefs.size() + B.pathLeafRefs.size();
    for (size_t h = 0; h < B.headLeaves.size(); ++h) {
        B.leafByPath[B.headLeaves[h].path].push_back(uint32_t(headBase + h));
    }
}

namespace {

bool
_LeafFlagged(const std::vector<char> &flags, int index)
{
    return index >= 0 && size_t(index) < flags.size() && flags[size_t(index)];
}

// RigExecBakedSampleLeaves' decision for one input: numbered, outside a
// bound walk, of \p pass's routing, and selected by one of its rules.
template <class T>
bool
_LeafResample(const RigExecBakedProgramImpl &B,
              const RigExecBakedInput<T> &input, RigExecBakedLeafPass pass,
              bool all, bool timeMoved, bool edited)
{
    if (input.leaf < 0) {
        return false;
    }
    // Bound walks are resolved by their consumer against current graph
    // values. Their source heads are sampled separately as raw leaves.
    if (input.walk >= 0)
        return false;
    const bool routed = bool(input.resolvedAttr);
    if ((pass == RigExecBakedLeafPass::BeforeHead && routed) ||
        (pass == RigExecBakedLeafPass::ChainRouted && !routed)) {
        return false;
    }
    const RigExecBakedLeafPool<T> &pool = B.leaves.Of<T>();
    const size_t k = size_t(input.leaf);
    const int o = input.overrideIndex;
    const bool walked = input.walk >= 0;
    // `all` first: a frozen job's handles are dead.
    return all || pool.mustSample[k] || (timeMoved && input.varying) ||
           _LeafFlagged(B.overridden, o) || _LeafFlagged(B.lastOverridden, o) ||
           (edited && _LeafFlagged(B.edited, o)) ||
           (walked && B.readerWalkMoved[size_t(input.walk)]);
}

// One input's visit, the same from the full sweep and the sparse pass:
// whether it was read. Each input owns its leaf and the read is pure, so
// the visiting order cannot change a value or a flag.
template <class T>
bool
_SampleLeafInput(RigExecBakedProgramImpl &B, const RigExecResolvedInputs &R,
                 RigExecBakedInput<T> &input, RigExecBakedLeafPass pass,
                 UsdTimeCode time, bool all, bool timeMoved, bool edited)
{
    using Stored = typename RigExecBakedLeafTraits<T>::Stored;
    ++B.leafVisits;
    if (!_LeafResample(B, input, pass, all, timeMoved, edited)) {
        return false;
    }
    RigExecBakedLeafPool<T> &pool = B.leaves.Of<T>();
    const size_t k = size_t(input.leaf);
    const bool walked = input.walk >= 0;
    pool.mustSample[k] = 0;
    ++B.leafSamples;
    // An upstream value on a hop reads the long way through the layer;
    // rule 8 marked the leaf when it was placed, moved or lifted.
    const Stored value =
        walked ? Stored(RigExecBakedReadWalked(B, input, B.overridden,
                                               &B.upstreamOn))
               : Stored(RigExecBakedRead(input, R, time, &B.overridden,
                                         &B.upstream, &B.upstreamOn));
    RigExecBakedNoteLeafWrite(&pool, k, !_LeafSame(value, pool.value[k]));
    pool.value[k] = value;
    if (walked && pool.changed[k]) {
        B.readerWalkChanged[size_t(input.walk)] = 1;
    }
    return true;
}

// \p fn(n) for every n with flags[n] != 0, ascending, testing eight bytes
// at a time.
template <class Fn>
void
_ForEachFlagged(const std::vector<char> &flags, Fn &&fn)
{
    const size_t n = flags.size();
    size_t i = 0;
    for (; i + 8 <= n; i += 8) {
        uint64_t word = 0;
        std::memcpy(&word, flags.data() + i, sizeof(word));
        if (word == 0) {
            continue;
        }
        for (size_t j = i; j < i + 8; ++j) {
            if (flags[j]) {
                fn(j);
            }
        }
    }
    for (; i < n; ++i) {
        if (flags[i]) {
            fn(i);
        }
    }
}

// \p fn on leaf \p id's input, cast to the type its pool names.
template <class Fn>
void
_WithLeafInput(RigExecBakedProgramImpl &B, uint32_t id, Fn &&fn)
{
    void *input =
        id < B.sparseInputs.input.size() ? B.sparseInputs.input[id] : nullptr;
    if (!input || id >= B.leafRefs.size()) {
        return;
    }
    switch (B.leafRefs[id].type) {
    case RigExecBakedLeafType::Double:
        fn(*static_cast<RigExecBakedInput<double> *>(input));
        return;
    case RigExecBakedLeafType::Float:
        fn(*static_cast<RigExecBakedInput<float> *>(input));
        return;
    case RigExecBakedLeafType::Int:
        fn(*static_cast<RigExecBakedInput<int> *>(input));
        return;
    case RigExecBakedLeafType::Bool:
        fn(*static_cast<RigExecBakedInput<bool> *>(input));
        return;
    case RigExecBakedLeafType::Token:
        fn(*static_cast<RigExecBakedInput<TfToken> *>(input));
        return;
    case RigExecBakedLeafType::Matrix4d:
        fn(*static_cast<RigExecBakedInput<GfMatrix4d> *>(input));
        return;
    case RigExecBakedLeafType::Vec3d:
        fn(*static_cast<RigExecBakedInput<GfVec3d> *>(input));
        return;
    case RigExecBakedLeafType::Vec3f:
        fn(*static_cast<RigExecBakedInput<GfVec3f> *>(input));
        return;
    }
}

// Builds `sparseInputs` for \p program unless it already describes it:
// one visitor traversal. False, leaving it empty, when some leaf id has no
// input to point at; the caller then sweeps.
bool
_SparseInputsReady(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedSparseInputs &S = B.sparseInputs;
    if (S.owner == &B && S.leafCount == B.leafRefs.size()) {
        return true;
    }
    S = RigExecBakedSparseInputs();
    const size_t n = B.leafRefs.size();
    S.input.assign(n, nullptr);
    // (override number, leaf id) of every input outside a walk.
    std::vector<std::pair<uint32_t, uint32_t>> numbered;
    size_t numbers = 0;
    bool complete = true;
    frozenDetail::_ForEachPatchableInput(B, [&](auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        if (input.leaf < 0) {
            return;
        }
        const RigExecBakedLeafPool<T> &pool = B.leaves.Of<T>();
        const size_t k = size_t(input.leaf);
        if (k >= pool.id.size() || pool.id[k] >= n ||
            B.leafRefs[pool.id[k]].type != RigExecBakedLeafTraits<T>::type) {
            complete = false;
            return;
        }
        const uint32_t id = pool.id[k];
        S.input[id] = &input;
        if (input.walk >= 0) {
            return;
        }
        if (input.varying) {
            S.varying.push_back(id);
        }
        if (input.overrideIndex >= 0) {
            numbered.emplace_back(uint32_t(input.overrideIndex), id);
            numbers = std::max(numbers, size_t(input.overrideIndex) + 1);
        }
    });
    for (const void *input : S.input) {
        complete = complete && input != nullptr;
    }
    if (!complete) {
        S = RigExecBakedSparseInputs();
        return false;
    }
    S.numberBegin.assign(numbers + 1, 0);
    for (const auto &entry : numbered) {
        ++S.numberBegin[size_t(entry.first) + 1];
    }
    for (size_t i = 0; i < numbers; ++i) {
        S.numberBegin[i + 1] += S.numberBegin[i];
    }
    S.numberLeaves.assign(numbered.size(), 0);
    std::vector<uint32_t> next(S.numberBegin.begin(), S.numberBegin.end() - 1);
    for (const auto &entry : numbered) {
        S.numberLeaves[next[size_t(entry.first)]++] = entry.second;
    }
    S.visited.assign(n, 0);
    S.owner = &B;
    S.leafCount = n;
    return true;
}

} // namespace

void
RigExecBakedSampleLeaves(RigExecBakedProgramImpl *program, UsdTimeCode time,
                         bool all, RigExecBakedLeafPass pass)
{
    RigExecBakedProgramImpl &B = *program;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    const bool first = pass != RigExecBakedLeafPass::ChainRouted;
    // Rule 6, carried into `mustSample`: the leaves under a routed override
    // whose value moved (placed, changed or lifted).
    if (first && B.routedOverrides != B.lastRoutedOverrides) {
        const auto markPath = [&B](const SdfPath &path) {
            const auto found = B.leafByPath.find(path);
            if (found != B.leafByPath.end()) {
                for (const uint32_t id : found->second) {
                    RigExecBakedMarkLeaf(&B, id);
                }
            }
        };
        for (const auto &[path, value] : B.routedOverrides) {
            const auto last = B.lastRoutedOverrides.find(path);
            if (last == B.lastRoutedOverrides.end() ||
                !(last->second == value)) {
                markPath(path);
            }
        }
        for (const auto &[path, value] : B.lastRoutedOverrides) {
            if (!B.routedOverrides.count(path)) {
                markPath(path);
            }
        }
        B.lastRoutedOverrides = B.routedOverrides;
    }
    // Rules 1 and 2: the caller's `all`, the first run, a moved stamp.
    all = all || B.runState != RigExecBakedRunState::Ran ||
          B.programStamp != B.lastProgramStamp;
    const bool timeMoved = time != B.lastTime;
    const bool edited = B.anyEdited;
    // Both passes of a Run decide alike: `all` is recomputed from the same
    // state, and the index built by the first stands for the second.
    const bool sparse = !all && B.sparseSampling && _SparseInputsReady(&B);
    if (first) {
        ++B.leafFlagCompiledRig;
        B.leaves.ForEach([sparse](auto &pool) {
            if (sparse) {
                for (const uint32_t k : pool.changedList) {
                    if (k < pool.changed.size()) {
                        pool.changed[k] = 0;
                    }
                }
            } else {
                std::fill(pool.changed.begin(), pool.changed.end(), char(0));
            }
            pool.changedList.clear();
        });
    }
    if (!sparse) {
        // The sweep relists every input it samples that is still marked.
        B.leaves.ForEach([](auto &pool) { pool.markedList.clear(); });
        frozenDetail::_ForEachPatchableInput(B, [&](auto &input) {
            using T = std::decay_t<decltype(input.constant)>;
            _SampleLeafInput(B, R, input, pass, time, all, timeMoved, edited);
            if (input.leaf < 0 || input.walk >= 0) {
                return;
            }
            RigExecBakedLeafPool<T> &pool = B.leaves.Of<T>();
            const size_t k = size_t(input.leaf);
            if (k < pool.mustSample.size() && pool.mustSample[k]) {
                pool.markedList.push_back(uint32_t(k));
            }
        });
        return;
    }
    // Every input a rule can select, each once per pass: a set `mustSample`
    // byte (rules 5, 6 and 8) through the marked lists; a moved time (rule
    // 3) through the varying inputs and the avars a patch promoted (the
    // only run-time writer of `varying`); an override now or at the last
    // run (rule 4) and an edit (rule 5) through their numbers.
    RigExecBakedSparseInputs &S = B.sparseInputs;
    const uint64_t stamp = ++S.pass;
    const auto visit = [&](uint32_t id) {
        if (id >= S.visited.size() || S.visited[id] == stamp) {
            return;
        }
        S.visited[id] = stamp;
        _WithLeafInput(B, id, [&](auto &input) {
            _SampleLeafInput(B, R, input, pass, time, all, timeMoved, edited);
        });
    };
    B.leaves.ForEach([&visit](auto &pool) {
        for (size_t i = 0; i < pool.markedList.size(); ++i) {
            const uint32_t k = pool.markedList[i];
            if (k < pool.id.size()) {
                visit(pool.id[k]);
            }
        }
    });
    if (timeMoved) {
        for (const uint32_t id : S.varying) {
            visit(id);
        }
        const RigExecBakedLeafPool<double> &avars = B.leaves.Of<double>();
        for (const size_t binding : B.promotedAvars) {
            if (binding >= B.avarConstantBindings.size()) {
                continue;
            }
            const int leaf = B.avarConstantBindings[binding].input.leaf;
            if (leaf >= 0 && size_t(leaf) < avars.id.size()) {
                visit(avars.id[size_t(leaf)]);
            }
        }
    }
    const auto visitNumber = [&](size_t number) {
        if (number + 1 >= S.numberBegin.size()) {
            return;
        }
        for (uint32_t i = S.numberBegin[number]; i < S.numberBegin[number + 1];
             ++i) {
            visit(S.numberLeaves[i]);
        }
    };
    _ForEachFlagged(B.overridden, visitNumber);
    _ForEachFlagged(B.lastOverridden, visitNumber);
    if (edited) {
        _ForEachFlagged(B.edited, visitNumber);
    }
    // RIGEXEC_VERIFY_SPARSE_SAMPLING: no input the pass left unvisited was
    // due for a read.
    if (B.verifySparseSampling) {
        size_t missed = 0;
        frozenDetail::_ForEachPatchableInput(B, [&](auto &input) {
            using T = std::decay_t<decltype(input.constant)>;
            if (input.leaf < 0) {
                return;
            }
            const RigExecBakedLeafPool<T> &pool = B.leaves.Of<T>();
            const size_t k = size_t(input.leaf);
            if (k < pool.id.size() && pool.id[k] < S.visited.size() &&
                S.visited[pool.id[k]] == stamp) {
                return;
            }
            if (_LeafResample(B, input, pass, all, timeMoved, edited)) {
                ++missed;
            }
        });
        if (missed) {
            B.sparseSamplingMismatches += missed;
            TF_VERIFY(false,
                      "%zu binding leaf(s) the sparse sample pass did not "
                      "visit were due for a read",
                      missed);
        }
    }
    // Keep the entries still marked whose input this function samples: a
    // routed one marked before BeforeHead stays for ChainRouted.
    B.leaves.ForEach([&S](auto &pool) {
        using T = typename std::decay_t<decltype(pool)>::Type;
        size_t kept = 0;
        for (size_t i = 0; i < pool.markedList.size(); ++i) {
            const uint32_t k = pool.markedList[i];
            if (k >= pool.mustSample.size() || !pool.mustSample[k] ||
                k >= pool.id.size() || pool.id[k] >= S.input.size()) {
                continue;
            }
            const auto *input =
                static_cast<const RigExecBakedInput<T> *>(S.input[pool.id[k]]);
            if (input && input->walk < 0) {
                pool.markedList[kept++] = k;
            }
        }
        pool.markedList.resize(kept);
    });
}

void
RigExecBakedComposeRestRange(RigExecBakedProgramImpl *program, int begin,
                             int end, bool trackMoves)
{
    BakedPoseHost host(program, nullptr);
    eval::RunRestComposeRange(host, begin, end, trackMoves);
}

void
RigExecBakedComposeLadderRange(RigExecBakedProgramImpl *program, int begin,
                               int end, bool trackMoves)
{
    BakedPoseHost host(program, nullptr);
    eval::RunLadderComposeRange(host, begin, end, trackMoves);
}

namespace {

// The ladder channels a RestCompose (\p rest) or LadderCompose reads for
// one slot, in a fixed order.
template <class Fn>
void
_ForEachLadderChannel(const RigExecBakedProgramImpl::Ladder &L, bool rest,
                      Fn &&fn)
{
    if (rest) {
        for (const RigExecBakedInput<double> &input : L.restAvars) {
            fn(input);
        }
        fn(L.restSpace);
        fn(L.interveningSpace);
    } else {
        fn(L.posedSpace);
        fn(L.defaultSpace);
        fn(L.parentSpace);
        fn(L.parentDefaultSpace);
        fn(L.avarDefaultSpace);
        fn(L.posedDefaultSpace);
        fn(L.rotationSign);
        for (const RigExecBakedInput<double> &input : L.defaultAvars) {
            fn(input);
        }
        fn(L.rotationOrder);
    }
}

} // namespace

void
RigExecBakedBuildRestSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecPathText pathText;
    const auto ranges = [](RigExecBakedSlotDomain domain,
                           const std::set<uint32_t> &slots,
                           std::vector<RigExecBakedSlotRange> *out) {
        for (const uint32_t slot : slots) {
            if (!out->empty() && out->back().domain == domain &&
                out->back().end == slot) {
                ++out->back().end;
            } else {
                out->push_back(RigExecBakedOne(domain, slot));
            }
        }
    };
    for (size_t g = 0; g < B.composeGroups.size(); ++g) {
        const RigExecBakedComposeGroup &group = B.composeGroups[g];
        const SdfPath &first = B.paths[size_t(group.begin)];
        for (const bool rest : {true, false}) {
            RigExecBakedStep step;
            step.isHead = true;
            step.kind = rest ? RigExecBakedStepKind::RestCompose
                             : RigExecBakedStepKind::LadderCompose;
            step.object = int(g);
            step.part = rest ? 0 : 1;
            step.label = std::string(rest ? "RestCompose " : "LadderCompose ") +
                         pathText(first);
            std::set<uint32_t> versions, rests, ladders, spaces;
            for (int i = group.begin; i < group.end; ++i) {
                const size_t slot = size_t(i);
                if (B.slotKind[slot] != RigExecBakedSlotKind::FirstFramePose) {
                    continue;
                }
                const auto &L = B.ladders[slot];
                for (size_t channel = 0; channel < 7; ++channel) {
                    if ((rest && channel != 0) || (!rest && channel == 0))
                        continue;
                    if (L.spaceValues[channel] >= 0)
                        spaces.insert(uint32_t(L.spaceValues[channel]));
                }
                const int parent = B.parent[slot];
                if (parent >= 0 &&
                    (parent < group.begin || parent >= group.end)) {
                    rests.insert(uint32_t(parent));
                    if (!rest) {
                        ladders.insert(uint32_t(parent));
                    }
                }
                _ForEachLadderChannel(
                    B.ladders[slot], rest, [&](const auto &input) {
                        if (input.walk >= 0) {
                            const RigExecBakedReaderWalk &reader =
                                B.readerWalks[size_t(input.walk)];
                            versions.insert(reader.versions.begin(),
                                            reader.versions.end());
                        }
                    });
            }
            if (!rest) {
                // The ladder reads the rests of its own slots.
                for (int i = group.begin; i < group.end; ++i) {
                    rests.insert(uint32_t(i));
                }
            }
            ranges(RigExecBakedSlotDomain::SpaceValue, spaces, &step.reads);
            ranges(RigExecBakedSlotDomain::PropertyResult, versions,
                   &step.reads);
            ranges(RigExecBakedSlotDomain::Rest, rests, &step.reads);
            ranges(RigExecBakedSlotDomain::Ladder, ladders, &step.reads);
            step.writes.push_back(RigExecBakedSlotRange{
                rest ? RigExecBakedSlotDomain::Rest
                     : RigExecBakedSlotDomain::Ladder,
                uint32_t(group.begin), uint32_t(group.end)});
            B.steps.push_back(std::move(step));
        }
    }
    const size_t N = B.paths.size();
    B.restChanged.assign(N, 0);
    B.ladderChanged.assign(N, 0);
    B.restMoved.clear();
    B.ladderMoved.clear();
}

void
RigExecBakedNoteRestLeaves(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedStep &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::RestCompose &&
            step.kind != RigExecBakedStepKind::LadderCompose) {
            continue;
        }
        step.bindingLeaves.clear();
        step.varyingLeaves = false;
        const RigExecBakedComposeGroup &group =
            B.composeGroups[size_t(step.object)];
        const bool rest = step.kind == RigExecBakedStepKind::RestCompose;
        for (int i = group.begin; i < group.end; ++i) {
            const size_t slot = size_t(i);
            if (B.slotKind[slot] != RigExecBakedSlotKind::FirstFramePose) {
                continue;
            }
            _ForEachLadderChannel(
                B.ladders[slot], rest, [&](const auto &input) {
                    using T = std::decay_t<decltype(input.constant)>;
                    if (input.leaf < 0) {
                        return;
                    }
                    step.bindingLeaves.push_back(
                        B.leaves.Of<T>().id[size_t(input.leaf)]);
                    step.varyingLeaves = step.varyingLeaves || input.varying;
                });
        }
    }
    // A notice between Build and the first run moves the stamp past this.
}

std::vector<RigExecBakedSlotRange>
RigExecBakedRequiredRestReads(const RigExecBakedProgramImpl &B,
                              const RigExecBakedStep &source)
{
    RigExecBakedStep step = source;
    step.reads.clear();
    const RigExecBakedStepIdentity identity = RigExecBakedIdentityOf(B, source);
    step.kind = identity.kind;
    step.object = identity.object;
    step.part = identity.part;
    const int N = int(B.paths.size());
    {
        std::set<uint32_t> rests, ladders;
        // Plain Xform slots have immutable identity rest/default tables.
        // Their sampled pose is a separate PoseBase/PoseFin value.
        const auto rest = [&B, &rests, N](int slot) {
            if (slot >= 0 && slot < N &&
                B.slotKind[size_t(slot)] ==
                    RigExecBakedSlotKind::FirstFramePose) {
                rests.insert(uint32_t(slot));
            }
        };
        const auto ladder = [&B, &ladders, N](int slot) {
            if (slot >= 0 && slot < N &&
                B.slotKind[size_t(slot)] ==
                    RigExecBakedSlotKind::FirstFramePose) {
                ladders.insert(uint32_t(slot));
            }
        };
        const auto version =
            [&ladder](const RigExecBakedProgramImpl::SpaceSwitch::FrameVersion
                          &read) {
                for (const int slot : read.recompose) {
                    ladder(slot);
                }
            };
        const int solverIndex = RigExecBakedSolverOf(B, step);
        switch (solverIndex >= 0 ? RigExecBakedStepKind::Solve : step.kind) {
        case RigExecBakedStepKind::ComposeSubtree: {
            if (step.object < 0 ||
                size_t(step.object) >= B.composeGroups.size())
                break;
            const RigExecBakedComposeGroup &group =
                B.composeGroups[size_t(step.object)];
            for (int slot = std::max(0, group.begin);
                 slot < std::min(N, group.end); ++slot) {
                ladder(slot);
            }
            for (const auto &ac : B.autoClavicles) {
                if (ac.slot < group.begin || ac.slot >= group.end)
                    continue;
                for (const auto &read : ac.frames) {
                    if (read.computation ==
                        SchemaComputationText::computeRestFrame)
                        rest(read.slot);
                    if (read.computation ==
                        SchemaComputationText::computeDefaultFrame)
                        ladder(read.slot);
                    for (int child : read.recompose)
                        ladder(child);
                }
            }
            for (const int parent : group.parentSlots) {
                ladder(parent);
            }
            for (int slot = std::max(0, group.begin);
                 slot < std::min(N, group.end); ++slot) {
                const int switchIndex =
                    size_t(slot) >= B.spaceSwitchBySlot.size()
                        ? -1
                        : B.spaceSwitchBySlot[size_t(slot)];
                if (switchIndex < 0 ||
                    size_t(switchIndex) >= B.spaceSwitches.size()) {
                    continue;
                }
                const RigExecBakedProgramImpl::SpaceSwitch &sw =
                    B.spaceSwitches[size_t(switchIndex)];
                if (size_t(slot) < B.parent.size())
                    ladder(B.parent[size_t(slot)]);
                ladder(sw.spaceSlot);
                for (const int source : sw.sourceSlots) {
                    ladder(source);
                }
                version(sw.parentRead);
                version(sw.spaceRead);
                for (const auto &read : sw.sourceReads) {
                    version(read);
                }
            }
            break;
        }
        case RigExecBakedStepKind::SpaceCheckpoint:
            if (step.object < 0 ||
                size_t(step.object) >= B.switchFrameContexts.size())
                break;
            for (const int slot :
                 B.switchFrameContexts[size_t(step.object)].recompose)
                ladder(slot);
            break;
        case RigExecBakedStepKind::Solve:
            if (size_t(solverIndex) >= B.solvers.size())
                break;
            for (const int slot : B.solvers[size_t(solverIndex)].restSlots) {
                rest(slot);
            }
            break;
        case RigExecBakedStepKind::Constraint: {
            if (step.object < 0 || size_t(step.object) >= B.walkSteps.size())
                break;
            const RigExecBakedProgramImpl::WalkStep &walk =
                B.walkSteps[size_t(step.object)];
            if (walk.solverBatch || walk.index < 0 ||
                size_t(walk.index) >= B.constraints.size()) {
                break;
            }
            const RigExecBakedProgramImpl::Constraint &c =
                B.constraints[size_t(walk.index)];
            if (!c.useAnimatedTs) {
                for (const int slot : c.targetSlots) {
                    rest(slot);
                }
            }
            if (c.type == SchemaTypeName::RigExecMatrixMover) {
                for (const int slot : c.sources) {
                    rest(slot);
                }
            }
            if (c.ikFkMatch) {
                for (size_t k = 0; k < c.sources.size(); ++k) {
                    if (k < 3 || k >= 5)
                        rest(c.sources[k]);
                }
                rest(c.spaceSlot);
            }
            ladder(c.spaceSlot);
            break;
        }
        case RigExecBakedStepKind::PropagateChunk: {
            if (step.object < 0 || size_t(step.object) >= B.commits.size())
                break;
            const auto &commit = B.commits[size_t(step.object)];
            for (const auto &pair : commit.propagate)
                for (int slot = pair.first; slot >= 0 && slot != pair.second;
                     slot = B.propParent[size_t(slot)])
                    ladder(slot);
            break;
        }
        case RigExecBakedStepKind::ProviderMatrix:
            rest(step.object);
            break;
        case RigExecBakedStepKind::FrameMatrix:
            if (step.object < 0 || size_t(step.object) >= B.frameRecords.size())
                break;
            rest(B.frameRecords[size_t(step.object)].slot);
            break;
        case RigExecBakedStepKind::PoseInterpolator: {
            if (step.object < 0 ||
                size_t(step.object) >= B.poseInterpolators.size())
                break;
            const RigExecBakedProgramImpl::PoseInterpolator &interpolator =
                B.poseInterpolators[size_t(step.object)];
            if (interpolator.driverSlot >= 0) {
                rest(interpolator.driverSlot);
                rest(interpolator.parentSlot);
                for (size_t e = 0; e < interpolator.extraDriverSlots.size();
                     ++e) {
                    rest(interpolator.extraDriverSlots[e]);
                    rest(interpolator.extraParentSlots[e]);
                }
            }
            break;
        }
        case RigExecBakedStepKind::Derived: {
            if (step.object < 0 || size_t(step.object) >= B.derivedIndex.size())
                break;
            const auto &[c, d] = B.derivedIndex[size_t(step.object)];
            if (c < 0 || size_t(c) >= B.chains.size() || d < 0 ||
                size_t(d) >= B.chains[size_t(c)].derived.size())
                break;
            const RigExecBakedProgramImpl::GeomRevision &revision =
                B.chains[size_t(c)].derived[size_t(d)].revision;
            rest(revision.transformSlot);
            rest(revision.transformSpaceSlot);
            rest(revision.carrySpaceSlot);
            break;
        }
        default:
            break;
        }
        for (const auto &[domain, slots] :
             {std::make_pair(RigExecBakedSlotDomain::Rest, &rests),
              std::make_pair(RigExecBakedSlotDomain::Ladder, &ladders)}) {
            for (const uint32_t slot : *slots) {
                if (!step.reads.empty() && step.reads.back().domain == domain &&
                    step.reads.back().end == slot) {
                    ++step.reads.back().end;
                } else {
                    step.reads.push_back(RigExecBakedOne(domain, slot));
                }
            }
        }
    }
    return step.reads;
}

void
RigExecBakedDeclareRestReads(RigExecBakedProgramImpl *program)
{
    for (auto &step : program->steps) {
        if (step.isHead)
            continue;
        const auto reads = RigExecBakedRequiredRestReads(*program, step);
        step.reads.insert(step.reads.end(), reads.begin(), reads.end());
    }
}

RigExecBakedProgramTesting::LadderTables
RigExecBakedProgramTesting::LadderTablesOf(const RigExecBakedProgram &program)
{
    const RigExecBakedProgramImpl &B = *program._impl;
    LadderTables T;
    T.restM = B.restM;
    T.restPts = B.restPts;
    T.restFrames = B.restFrames;
    T.selfD = B.selfD;
    T.parentDinv = B.parentDinv;
    T.rotOrder = B.rotOrder;
    T.restRoundTrip = B.restRoundTrip;
    T.defaultRoundTrip = B.defaultRoundTrip;
    T.posedAuthored = B.posedAuthored;
    T.posedAuthoredM = B.posedAuthoredM;
    return T;
}

void
RigExecBakedBuildAvarSteps(RigExecBakedProgramImpl *program)
{
    auto &B = *program;
    std::vector<int> origin(B.paths.size(), -1);
    for (size_t o = 0; o < B.opNodeOrigins.size(); ++o)
        if (B.opNodeOrigins[o].family == RigExecBakedOpFamily::AvarInputs)
            origin[size_t(B.opNodeOrigins[o].object)] = int(o);
    RigExecPathText pathText;
    for (size_t slot = 0; slot < B.paths.size(); ++slot) {
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::OpNode;
        step.object = origin[slot];
        step.maxDiagnostics = 1;
        step.label = "AvarInputs " + pathText(B.paths[slot]);
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::Avars, slot));
        B.steps.push_back(std::move(step));
    }
}
void
RigExecBakedIndexAvarBindings(RigExecBakedProgramImpl *program)
{
    auto &B = *program;
    size_t providers = B.avarConstants.size() / 11;
    for (const auto *bindings : {&B.avarBindings, &B.avarConstantBindings})
        for (const auto &binding : *bindings)
            providers = std::max(providers, binding.slot / 11 + 1);
    const auto index =
        [providers](
            const std::vector<RigExecBakedProgramImpl::AvarBinding> &bindings,
            std::vector<uint32_t> *begin) {
            // Contiguous runs per provider need the list sorted by provider;
            // ascending flat slots give that, and Build appends them so.
            TF_VERIFY(std::is_sorted(
                bindings.begin(), bindings.end(),
                [](const auto &a, const auto &b) { return a.slot < b.slot; }));
            begin->assign(providers + 1, 0);
            for (const auto &binding : bindings)
                ++(*begin)[binding.slot / 11 + 1];
            for (size_t p = 0; p < providers; ++p)
                (*begin)[p + 1] += (*begin)[p];
        };
    index(B.avarBindings, &B.avarBindingBegin);
    index(B.avarConstantBindings, &B.avarConstantBindingBegin);
}
void
RigExecBakedRunSolverSources(RigExecBakedProgramImpl *program, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedProgramImpl::Solver &s : B.solvers) {
        if (!s.ribbonPointsVarying || !s.ribbonPointsQuery.IsValid()) {
            continue;
        }
        // The driver attribute's value at this generation's time; a failed read
        // leaves the array empty. The pinned query answers the same value the
        // attribute does; only the resolve is cached.
        VtVec3fArray live;
        s.ribbonPointsQuery.Get(&live, time);
        s.lastRibbonPoints.swap(s.ribbonPoints);
        s.ribbonPoints.assign(live.cbegin(), live.cend());
        // By VALUE, never "the time moved": a curve keyed on two frames
        // holds the same points across most of a sweep, and comparing is
        // what lets the ribbon's whole cone sit out those frames.
        s.ribbonPointsDirty = s.ribbonPoints != s.lastRibbonPoints;
    }
}

namespace {

/// Slot \p i's avars as one matrix, the way computations.cpp composes them.
GfMatrix4d
_ComposeAvarsOf(const RigExecBakedProgramImpl &B, int i)
{
    const double *a = &B.avars[size_t(i) * 11];
    const double units = a[10];
    // A volume weight's placement is RIGID: its shape is
    // inputs:scaleX/Y/Z's alone, so the transform-scale avars are read and
    // discarded here rather than zeroed at bake -- exec never binds them at
    // all, and a captured zero would be walked straight past by an override
    // or an animated channel. The authored-posed:space branch reads no avar
    // at all, so a volume that takes it discards the scale for free, which
    // is what exec does with the same rig.
    const bool noScale = B.noScaleAvars[size_t(i)] != 0;
    // avars:rotationSign, applied to the avar exactly where computations.cpp
    // applies it, so a mirrored limb composes the same numbers on both paths.
    const unsigned sign =
        size_t(i) < B.rotationSign.size() ? B.rotationSign[size_t(i)] : 0u;
    const double sx = RigExecRotationSignFromMask(sign, 0);
    return RigExecBakedComposeAvars(a[0] * units, a[1] * units, a[2] * units,
                                    noScale ? 1.0 : a[3], noScale ? 1.0 : a[4],
                                    noScale ? 1.0 : a[5], a[6] * sx,
                                    a[7] * RigExecRotationSignFromMask(sign, 1),
                                    a[8] * RigExecRotationSignFromMask(sign, 2),
                                    a[9] * sx, B.rotOrder[size_t(i)]);
}

/// The ordinary compose of slot \p i against \p parentPosed, which is what
/// exec answers for a provider no switch has replaced.
RigExecPointFrame
_ComposeUnswitched(const RigExecBakedProgramImpl &B, int i,
                   const GfMatrix4d &parentPosed)
{
    if (B.posedAuthored[size_t(i)]) {
        // A non-identity authored posed:space is the pose: exec returns its
        // frame and reads neither the avars nor the parent.
        return RigExecFrameFromMatrix(B.posedAuthoredM[size_t(i)]);
    }
    return RigExecFrameFromMatrix(RigExecComposeUnswitchedPoseMatrix(
        _ComposeAvarsOf(B, i), B.posedD[size_t(i)], B.parentDinv[size_t(i)],
        B.parentSpaceAuthored[size_t(i)] ? B.parentSpaceM[size_t(i)]
                                         : parentPosed));
}

/// _SpaceFromFrame: an unusable frame selects the NaN sentinel, so the
/// failure survives into every descendant instead of being scrubbed into a
/// plausible identity.
GfMatrix4d
_SpaceOfFrame(const RigExecPointFrame &frame)
{
    GfMatrix4d space(1.0);
    if (!frame.IsValid() || frame.IsDegenerate() ||
        !RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                               &space)) {
        space = GfMatrix4d(1.0);
        space[3][0] = std::numeric_limits<double>::quiet_NaN();
    }
    return space;
}

/// Reads an explicit composed provider or a separately produced checkpoint.
/// The consumer performs no recomposition or source walk.
GfMatrix4d
_ReadFrameVersion(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::SpaceSwitch::FrameVersion &read)
{
    if (read.context >= 0)
        return B.switchFrames[size_t(read.context)];
    return read.anchor >= 0 ? B.posedM[size_t(read.anchor)] : GfMatrix4d(1.0);
}

} // namespace

bool
RigExecBakedEvalFrameRecord(const RigExecBakedProgramImpl &B,
                            const RigExecBakedFrameRecord &record,
                            GfMatrix4d *matrix)
{
    *matrix = GfMatrix4d(1.0);
    const RigExecBakedCommit &commit = B.commits[size_t(record.commit)];
    if (!eval::FrameRecordRecorded(commit.solverOutput, record.position,
                                   record.target, commit.present.data(),
                                   commit.present.size(), commit.recordAfter,
                                   commit.recordEveryTarget)) {
        return false;
    }
    const RigExecPointFrame &frame = B.fin[size_t(record.version)];
    const RigExecPointFrame &rest = B.restFrames[size_t(record.slot)];
    return eval::FrameRecordMatrix<BakedPoseMath>(
        rest.points, rest.flags, frame.points, frame.flags, matrix);
}

void
RigExecBakedRunPoseStep(RigExecBakedProgramImpl *program,
                        RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    // Every per-frame read goes through `rd`: one input's leaf, sampled in
    // the prologue the way this generation must read it (through the
    // resolved inputs while an override stands on it, through the pinned
    // query otherwise).
    const auto rd = [&B](const auto &input) {
        return RigExecBakedLeafRead(B, input);
    };

    switch (step->kind) {
    case RigExecBakedStepKind::ComposeSubtree: {
        const RigExecBakedComposeGroup &group =
            B.composeGroups[size_t(step->object)];
        BakedPoseHost host(program, step);
        for (int i = group.begin; i < group.end; ++i) {
            if (!host.inputs().isPoseSlot(size_t(i))) {
                // An xform-derived slot is not composed from avars.
                continue;
            }
            host.state().beforeSlot(eval::StepKind::ComposeSubtree, group.begin,
                                    i);
            if (!host.inputs().providerActive(size_t(i))) {
                eval::PublishComposedSlotUnavailable(host, size_t(i));
                continue;
            }
            const int switchIndex = B.spaceSwitchBySlot.empty()
                                        ? -1
                                        : B.spaceSwitchBySlot[size_t(i)];
            if (switchIndex < 0) {
                const int parent = host.inputs().parent(size_t(i));
                GfMatrix4d parentPosed(1.0);
                if (parent >= 0 &&
                    !host.inputs().posedMatrix(size_t(parent), &parentPosed)) {
                    step->diagnostics.push_back(
                        "compose slot " + std::to_string(i) + " names no slot");
                    continue;
                }
                std::array<GfVec3d, 4> points;
                unsigned flags = 0;
                if (!eval::ComposeSlotFrame(host, size_t(i), parentPosed,
                                            &points, &flags)) {
                    step->diagnostics.push_back(
                        "compose slot " + std::to_string(i) + " names no slot");
                    continue;
                }
                host.state().writeBase(size_t(i), points, flags);
            } else if (B.posedAuthored[size_t(i)]) {
                // A non-identity authored posed:space is the pose: exec
                // returns its frame and reads neither the avars nor the
                // parent, so neither does this.
                B.base[size_t(i)] =
                    RigExecFrameFromMatrix(B.posedAuthoredM[size_t(i)]);
            } else {
                auto &sw = B.spaceSwitches[size_t(switchIndex)];
                auto &input = sw.kernelInputs;
                input.avars = _ComposeAvarsOf(B, i);
                input.posedDefault = B.posedD[size_t(i)];
                input.parentDefaultInverse = B.parentDinv[size_t(i)];
                input.parentPosed = _ReadFrameVersion(B, sw.parentRead);
                input.parentDefault =
                    B.parent[size_t(i)] >= 0
                        ? B.defaultRoundTrip[size_t(B.parent[size_t(i)])]
                        : GfMatrix4d(1.0);
                input.active = 0.0;
                if (sw.tokenIndex) {
                    const TfToken selected = rd(sw.activeTokenInput);
                    const auto label =
                        std::find(sw.labels.begin(), sw.labels.end(), selected);
                    if (label != sw.labels.end())
                        input.active = double(label - sw.labels.begin());
                } else
                    input.active = rd(sw.activeInput);
                input.hasCarry = sw.spaceSlot >= 0;
                if (input.hasCarry) {
                    input.spaceDefault =
                        B.defaultRoundTrip[size_t(sw.spaceSlot)];
                    input.spacePosed = _ReadFrameVersion(B, sw.spaceRead);
                }
                for (size_t source = 0; source < sw.sourceSlots.size();
                     ++source) {
                    const int slot = sw.sourceSlots[source];
                    auto &value = input.sources[source];
                    value.world = slot < 0;
                    if (slot >= 0) {
                        value.defaultSpace = B.defaultRoundTrip[size_t(slot)];
                        value.posedSpace =
                            _ReadFrameVersion(B, sw.sourceReads[source]);
                    }
                }
                if (!RigExecRunSpaceSwitch(sw.kernelRecord, input,
                                           &B.base[size_t(i)])) {
                    B.base[size_t(i)].flags = 0;
                    sw.matchValid = false;
                    step->diagnostics.push_back(
                        "space switch has no source for " +
                        (*B.pathTexts)[size_t(i)]);
                } else {
                    RigExecBakedRunSpaceMatch(
                        B, sw, i, input, B.base[size_t(i)], &step->diagnostics);
                }
            }
            const int autoIndex = (size_t(i) < B.autoClavicleBySlot.size()
                                       ? B.autoClavicleBySlot[size_t(i)]
                                       : -1);
            if (autoIndex >= 0) {
                auto &ac = B.autoClavicles[size_t(autoIndex)];
                ac.values.Publish(0, B.base[size_t(i)]);
                for (const auto &read : ac.frames) {
                    RigExecPointFrame frame;
                    if (read.computation ==
                        SchemaComputationText::computeRestFrame)
                        frame = B.restFrames[size_t(read.slot)];
                    else if (read.computation ==
                             SchemaComputationText::computeDefaultFrame)
                        frame = RigExecFrameFromMatrix(
                            B.defaultRoundTrip[size_t(read.slot)]);
                    else if (B.paths[size_t(read.slot)].HasPrefix(
                                 B.paths[size_t(i)])) {
                        frame = B.base[size_t(i)];
                        for (int child : read.recompose)
                            frame = _ComposeUnswitched(B, child,
                                                       _SpaceOfFrame(frame));
                    } else
                        frame = B.fin[size_t(read.slot)];
                    ac.values.Publish(read.value, frame);
                }
                for (const auto &read : ac.scalars) {
                    if (read.isFloat)
                        ac.values.Publish(read.value, rd(read.narrow), false,
                                          true);
                    else
                        ac.values.Publish(read.value, rd(read.wide), false,
                                          true);
                }
                std::string invalid;
                if (!RigExecDispatchRunAutoClavicle(ac.operation, &ac.values,
                                                    &invalid))
                    step->diagnostics.push_back(invalid);
                if (const auto *frame =
                        ac.values.Read<RigExecPointFrame>(ac.operation.output))
                    B.base[size_t(i)] = *frame;
            }
            eval::PublishFinPosedM(host, size_t(i));
        }
        return;
    }

    default:
        return;
    }
}

bool
RigExecBakedPublishPose(RigExecBakedProgramImpl *program, RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedEnsureEpilogueIndex(&B);
    if (B.epilogue.verify) {
        RigExecBakedVerifyEpilogueIndex(&B);
    }
    // SCC-excluded solvers did not attempt publication this generation.
    // The canonical operation inventory is the authority, not authored bindings.
    const std::vector<char> &aliveSolver = B.epilogue.aliveSolver;
    // Chain/part inventory is reported before pose diagnostics.
    RigExecBakedAppendStepLines(&B, RigExecBakedStepLines::Property,
                                &pose->diagnostics);
    // The walk's diagnostics, in step order: commit lines, constraint lines,
    // world-up lines. They were pushed into the pose as the walk produced them;
    // they are replayed here instead, so that no step body ever touches the
    // generation and the order is program order rather than completion order.
    // The pose interpolators' lines come after the joint block; replayed
    // below.
    RigExecBakedAppendStepLines(&B, RigExecBakedStepLines::Walk,
                                &pose->diagnostics);

    // One line per failing writer, walk order, stable-sorted by joint; "fell
    // back to its rest chain" only when no writer published. Merged
    // here, not per step: the order belongs to the whole run. Paths come from
    // Build's texts: SdfPath::GetString() interns under a lock.
    struct FallbackJoint {
        SdfPath joint;
        size_t jointSlot;
        const std::string *writerText;
        int element;
    };
    std::vector<FallbackJoint> fallbackJoints;
    std::map<SdfPath, const std::string *> lastPublishingWriter;
    for (const RigExecBakedProgramImpl::WalkStep &walk : B.walkSteps) {
        if (!walk.solverBatch) {
            continue;
        }
        for (const int si : walk.batchSolvers) {
            if (!aliveSolver[size_t(si)])
                continue;
            const RigExecBakedProgramImpl::Solver &s = B.solvers[size_t(si)];
            for (size_t k = 0; k < s.outputs.size(); ++k) {
                const size_t jointSlot = size_t(s.outputs[k].first);
                const SdfPath &jointPath = B.paths[jointSlot];
                if (k < s.outPresent.size() && s.outPresent[k]) {
                    lastPublishingWriter[jointPath] = &s.pathText;
                    continue;
                }
                int element = -1;
                const auto binding = (*B.jointSolverBinding).find(jointPath);
                if (binding != (*B.jointSolverBinding).end()) {
                    for (const auto &[writer, writerElement] :
                         binding->second) {
                        if (writer == s.path) {
                            element = writerElement;
                            break;
                        }
                    }
                }
                fallbackJoints.push_back(
                    {jointPath, jointSlot, &s.pathText, element});
            }
        }
    }
    std::stable_sort(fallbackJoints.begin(), fallbackJoints.end(),
                     [](const FallbackJoint &a, const FallbackJoint &b) {
                         return a.joint < b.joint;
                     });
    for (const FallbackJoint &fallback : fallbackJoints) {
        const auto kept = lastPublishingWriter.find(fallback.joint);
        pose->diagnostics.push_back(
            "solver " + *fallback.writerText + " published no element " +
            std::to_string(fallback.element) + " for joint " +
            (*B.pathTexts)[fallback.jointSlot] + "; " +
            (kept == lastPublishingWriter.end()
                 ? std::string("joint fell back to its rest chain")
                 : "the joint keeps the frame " + *kept->second + " left"));
    }

    // The plain Xformables a constraint targets, in slot (== path) order,
    // between the fallback-joint lines above and the joint publication's
    // "degenerate final frame" lines below: diagnostics are compared in order,
    // so the position of this loop is part of the answer.
    for (size_t k = 0; k < B.xformSlots.size(); ++k) {
        const size_t slot = size_t(B.xformSlots[k]);
        const RigExecPointFrame &frame = B.fin[size_t(B.finLast[slot])];
        if (!RigExecBakedUsable(frame)) {
            pose->diagnostics.push_back(
                "constraint target " + (*B.pathTexts)[slot] +
                " has an invalid final frame; transform omitted");
            continue;
        }
        GfMatrix4d revised(1.0);
        if (RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                                  &revised)) {
            // Slot order is path order and no slot is named twice, so both
            // maps are filled strictly ascending from empty.
            RigExecBakedEmplace(&pose->providerXforms, true, B.paths[slot],
                                revised);
            RigExecBakedEmplace(&pose->providerBaseXforms, true, B.paths[slot],
                                B.xformBase[k]);
        }
    }

    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedPublish", "baked");
        // A frame publishes its keys into five ordered maps. The two
        // passes below split that walk in half: the first decides, in the
        // publication list's own order, which joints publish a matrix and
        // which say why they do not -- the order those lines have always
        // been in -- and the second fills the maps in PATH order, where
        // every key belongs immediately past the one before it and a hint at
        // the map's end makes the insertion one comparison.
        // Deciding first also settles the bail: a frame that cannot publish
        // returns before a single key is inserted, never with the maps half
        // filled. The caller drops the pose either way.
        for (size_t k = 0; k < B.jointPaths.size(); ++k) {
            const int slot = B.jointSlots[k];
            const RigExecPointFrame &finalFrame =
                B.fin[size_t(B.finLast[size_t(slot)])];
            // The point frame is the status bearer; publishing an identity
            // matrix for a degenerate frame would let a matrix-only consumer
            // deform with a plausible-but-wrong transform.
            if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {

                B.jointMatrixPublished[k] = 1;
            } else {
                B.jointMatrixPublished[k] = 0;
                pose->diagnostics.push_back(
                    "joint " + (*B.pathTexts)[size_t(slot)] +
                    " has a degenerate final frame; matrix omitted");
            }
        }
        const bool jointsInOrder = B.jointPathsAscending;
        for (const int index : B.jointPublishOrder) {
            const size_t k = size_t(index);
            const int slot = B.jointSlots[k];
            RigExecBakedEmplace(&pose->jointFramesBase, jointsInOrder,
                                B.jointPaths[k],
                                B.base[size_t(B.baseLast[size_t(slot)])]);
            RigExecBakedEmplace(&pose->jointFramesFinal, jointsInOrder,
                                B.jointPaths[k],
                                B.fin[size_t(B.finLast[size_t(slot)])]);
            if (B.jointMatrixPublished[k]) {
                // A skipped key leaves the hint at the last one that landed,
                // which is still the largest in the map: omitting entries
                // keeps the sequence ascending.
                RigExecBakedEmplace(&pose->jointMatricesFinal, jointsInOrder,
                                    B.jointPaths[k],
                                    B.finalMatrix[size_t(slot)]);
            }
        }
        // Most controls are animator inputs and publish their base frame; a
        // control a constraint names publishes the revised one. Both are the
        // LAST version of the same slot here, because the walk wrote the
        // revision into a version of it.
        for (const int index : B.controlPublishOrder) {
            const size_t k = size_t(index);
            RigExecBakedEmplace(
                &pose->controlFrames, B.controlPathsAscending,
                B.controlPaths[k],
                B.fin[size_t(B.finLast[size_t(B.controlSlots[k])])]);
        }
        // IK/FK matches: their values, and the half they present.
        RigExecBakedPresentIkFkMatches(B, pose);
        RigExecBakedPresentSpaceMatches(B, pose);
    }

    // Observational solver guides: the published array is the aggregate the
    // walk produced. Read from the aggregates HERE, under the
    // runtime toggle, rather than cached in a step: either half of the toggle
    // can move without the compiled rig moving.
    if (*B.solverGuidesEnabled) {
        for (const int index : B.solverPublishOrder) {
            const auto &[solverPath, si] = B.solverArrays[size_t(index)];
            if (!aliveSolver[size_t(si)])
                continue;
            RigExecBakedEmplace(&pose->solverFrames, B.solverArraysAscending,
                                solverPath, B.aggregates[size_t(si)].frames);
        }
    }

    // Preserve diagnostic publication order: interpolators follow joint,
    // control and guide publications and precede geometry diagnostics.
    RigExecBakedAppendStepLines(&B, RigExecBakedStepLines::Interpolator,
                                &pose->diagnostics);

    // Property-domain results, in the same map as the point chains: a
    // consumer tells them apart by the type the VtValue holds.
    // propertyResults is itself an ordered map and movedProperties is empty
    // until here, so this walk is always ascending -- no flag to consult.
    for (const auto &[target, value] : B.propertyResults) {
        pose->movedProperties.emplace_hint(pose->movedProperties.end(), target,
                                           value);
    }
    // The pose weights, published into both movedProperties, which makes the
    // number observable to a host, a test and the picker, and the generation's
    // resolved inputs, which a consumer outside the program (the standalone
    // reader of a blend input's connection) resolves through after the frame.
    for (size_t k = 0; k < B.poseWeightPaths.size(); ++k) {
        const VtValue value(B.poseWeights[k]);
        pose->movedProperties[B.poseWeightPaths[k]] = value;
        B.resolvedInputs->SetProperty(B.poseWeightPaths[k], value);
    }
    return true;
}

} // namespace rigExec

namespace rigExec {

void
RigExecBakedResetAvars(RigExecBakedProgramImpl &B, uint32_t slot)
{
    std::fill(B.avars.begin() + size_t(slot) * 11,
              B.avars.begin() + (size_t(slot) + 1) * 11, 0.0);
}

void
RigExecBakedResetPoseBase(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.base[slot] = RigExecBakedSetAsideFrame();
}

void
RigExecBakedResetPoseFin(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.fin[slot] = RigExecBakedSetAsideFrame();
}

void
RigExecBakedResetPosedM(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.posedM[slot] = RigExecBakedSetAsideMatrix();
}

void
RigExecBakedResetFinalMatrix(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.finalMatrix[slot] = RigExecBakedSetAsideMatrix();
}

void
RigExecBakedResetBaseMatrix(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.baseMatrix[slot] = RigExecBakedSetAsideMatrix();
}

void
RigExecBakedResetSwitchFrame(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.switchFrames[slot] = RigExecBakedSetAsideMatrix();
}

void
RigExecBakedResetAggregate(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.aggregates[slot] = {};
}

void
RigExecBakedResetCandidates(RigExecBakedProgramImpl &B, uint32_t slot)
{
    std::fill(B.solvers[slot].outPresent.begin(),
              B.solvers[slot].outPresent.end(), char(0));
    std::fill(B.solvers[slot].outFrames.begin(),
              B.solvers[slot].outFrames.end(), RigExecBakedSetAsideFrame());
}

void
RigExecBakedResetCommitTable(RigExecBakedProgramImpl &B, uint32_t slot)
{
    auto &commit = B.commits[slot];
    commit.abandoned = true;
    std::fill(commit.present.begin(), commit.present.end(), char(0));
    std::fill(commit.frames.begin(), commit.frames.end(),
              RigExecBakedSetAsideFrame());
}

void
RigExecBakedResetCommitDelta(RigExecBakedProgramImpl &B, uint32_t slot)
{
    std::fill(B.commits[slot].deltaOk.begin(), B.commits[slot].deltaOk.end(),
              char(0));
}

void
RigExecBakedResetCommitStaging(RigExecBakedProgramImpl &B, uint32_t slot)
{
    for (auto &commit : B.commits)
        if (commit.split && slot >= uint32_t(commit.stagingBase) &&
            uint64_t(slot) - uint32_t(commit.stagingBase) <
                commit.staged.size()) {
            const auto i = slot - uint32_t(commit.stagingBase);
            commit.staged[i] = RigExecBakedSetAsideFrame();
            commit.outcome[i] =
                uint8_t(RigExecBakedPropagateOutcome::NoCandidate);
            return;
        }
}

void
RigExecBakedResetConstraintDelta(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.deltaPresent[slot] = 0;
    B.deltaValues[slot] = GfMatrix4d(1);
}

void
RigExecBakedResetPoseWeight(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.poseWeights[slot] = 0;
}

void
RigExecBakedResetFrameMatrix(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.frameMatrixValid[slot] = 0;
    B.frameMatrix[slot] = GfMatrix4d(1);
}

void
RigExecBakedResetRest(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.restFrames[slot] = RigExecBakedSetAsideFrame();
    B.restM[slot] = GfMatrix4d(1);
    B.restPts[slot] = {};
}

void
RigExecBakedResetLadder(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.selfD[slot] = B.parentDinv[slot] = B.restRoundTrip[slot] =
        B.defaultRoundTrip[slot] = GfMatrix4d(1);
    B.posedAuthored[slot] = B.parentSpaceAuthored[slot] = 0;
    B.posedAuthoredM[slot] = B.posedD[slot] = B.parentSpaceM[slot] =
        GfMatrix4d(1);
    B.rotOrder[slot] = TfToken();
    B.rotationSign[slot] = 0;
}

bool
RigExecBakedPathIdentity(const RigExecBakedProgramImpl &B, uint32_t slot,
                         RigExecPathText &text, std::string *out)
{
    if (slot >= B.paths.size())
        return false;
    *out += text(B.paths[slot]);
    return true;
}

} // namespace rigExec
