#include "rigExecSchema/schemaNames.h"
#include "solverSceneTranslation.h"
#include "sceneInputTypes.h"
#include "startFrameDerivation.h"
#include "skeleton2dTranslation.h"
#include "jointGroupTranslation.h"
#include "rigExec/pathText.h"
#include "rigExec/solverKernels.h"
#include "rigExec/types.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/gf/math.h"
#include "pxr/base/tf/staticTokens.h"
#include <type_traits>
namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    ((rigExecJointElements,SchemaAttrText::rigExecJointElements)) ((rigExecScaleCalibration,SchemaAttrText::rigExecScaleCalibration))
    ((rigExecWeights,SchemaAttrText::rigExecWeights)) ((rigExecCount,SchemaAttrText::rigExecCount)) (points)
    ((rigExecDriverCurve,SchemaAttrText::rigExecDriverCurve)) (base) ((rigExecVolumeWeights,SchemaAttrText::rigExecVolumeWeights))
    ((rigExecControlSpace,SchemaAttrText::rigExecControlSpace)) ((rigExecSegmentScale,SchemaAttrText::rigExecSegmentScale)) (none)
    ((rigExecStretchPolicy,SchemaAttrText::rigExecStretchPolicy)) (uniformSegments)
    ((rigExecScaleBlend,SchemaAttrText::rigExecScaleBlend)) ((rigExecRotationBlend,SchemaAttrText::rigExecRotationBlend)) (shortestArc)
    ((rigExecRestLength,SchemaAttrText::rigExecRestLength)) ((rigExecRootTangent,SchemaAttrText::rigExecRootTangent))
    ((rigExecJoints,SchemaAttrText::rigExecJoints)) ((rigExecControls,SchemaAttrText::rigExecControls))
    ((rigExecStartFrame,SchemaAttrText::rigExecStartFrame)) ((rigExecRootControl,SchemaAttrText::rigExecRootControl))
    ((rigExecEffectorControl,SchemaAttrText::rigExecEffectorControl)) ((rigExecPoleControl,SchemaAttrText::rigExecPoleControl))
    ((rigExecSpace,SchemaAttrText::rigExecSpace)) ((rigExecInputA,SchemaAttrText::rigExecInputA)) ((rigExecInputB,SchemaAttrText::rigExecInputB))
    ((rigExecStart,SchemaAttrText::rigExecStart)) ((rigExecEnd,SchemaAttrText::rigExecEnd)) ((rigExecMidControl,SchemaAttrText::rigExecMidControl))
    ((rigExecEndControl,SchemaAttrText::rigExecEndControl)) ((rigExecSpaceMatrix,SchemaAttrText::rigExecSpaceMatrix))
    ((rigExecPreferredBendRadians,SchemaAttrText::rigExecPreferredBendRadians))
    ((rigExecUpperLengthOffset,SchemaAttrText::rigExecUpperLengthOffset))
    ((rigExecLowerLengthOffset,SchemaAttrText::rigExecLowerLengthOffset)) ((inputsStretch,SchemaAttrText::inputsStretch))
    ((inputsSoftness,SchemaAttrText::inputsSoftness)) ((inputsPin,SchemaAttrText::inputsPin)) ((inputsUpperScale,SchemaAttrText::inputsUpperScale))
    ((inputsLowerScale,SchemaAttrText::inputsLowerScale)) ((inputsSoftDistance,SchemaAttrText::inputsSoftDistance))
    ((inputsTwist,SchemaAttrText::inputsTwist)) ((inputsWeight,SchemaAttrText::inputsWeight)) ((inputsTwistTurns,SchemaAttrText::inputsTwistTurns))
    ((rigExecSampleCount,SchemaAttrText::rigExecSampleCount)) ((inputsPreserveVolume,SchemaAttrText::inputsPreserveVolume))
    ((inputsMidFollowWeight,SchemaAttrText::inputsMidFollowWeight)) ((inputsRoll,SchemaAttrText::inputsRoll))
    ((inputsMinLengthRatio,SchemaAttrText::inputsMinLengthRatio))
);
namespace {
bool
Fail(std::string *error, const std::string &text)
{
    if (error)
        *error = text;
    return false;
}
} // namespace
bool
RigExecTranslateSceneSolver(const RigExecSceneDescriptors &scene,
                            const SdfPath &path,
                            const std::map<SdfPath, RigExecPointFrame> &rests,
                            RigExecSceneSolverDescriptor *output,
                            std::string *error)
{
    if (!output)
        return Fail(error, "null solver descriptor destination");
    const RigExecSceneCompileInputs source(scene);
    const auto *node = source.Node(path);
    if (!node || !node->fact.active ||
        node->domain != RigExecSceneDomain::Solver)
        return Fail(error, "missing active solver: " + path.GetString());
    RigExecSceneSolverDescriptor result;
    result.path = path;
    auto &s = result.record;
    const auto type = node->fact.type;
    if (type == SchemaTypeName::RigExecFkChain)
        s.kind = RigExecSolverKind::FkChain;
    else if (type == SchemaTypeName::RigExecTwoBoneIk)
        s.kind = RigExecSolverKind::TwoBoneIk;
    else if (type == SchemaTypeName::RigExecBlendPointFrames)
        s.kind = RigExecSolverKind::BlendPointFrames;
    else if (type == SchemaTypeName::RigExecTwistDistribution)
        s.kind = RigExecSolverKind::TwistDistribution;
    else if (type == SchemaTypeName::RigExecRibbon)
        s.kind = RigExecSolverKind::Ribbon;
    else if (type == SchemaTypeName::RigExecSplineIk)
        s.kind = RigExecSolverKind::SplineIk;
    else if (type == SchemaTypeName::RigExecSkeleton2d)
        s.kind = RigExecSolverKind::Skeleton2d;
    else
        return Fail(error, "unsupported solver: " + type.GetString());
    auto targets = [&](const TfToken &name) {
        return source.Targets(path.AppendProperty(name));
    };
    auto providerExists = [&](const SdfPath &provider) {
        const auto *fact = source.Node(provider);
        return fact && fact->domain == RigExecSceneDomain::Provider;
    };
    // Numerical seed availability cannot change structural solver cardinality.
    auto rest = [&](const SdfPath &provider) {
        const auto found = rests.find(provider);
        if (found != rests.end())
            return found->second;
        RigExecPointFrame unavailable;
        unavailable.flags = 0;
        return unavailable;
    };
    auto first = [&](const TfToken &name, bool provider) {
        const auto list = targets(name);
        return list.empty() || (provider && !providerExists(list.front()))
                   ? SdfPath{}
                   : list.front();
    };
    auto read = [&](const SdfPath &property, UsdTimeCode time, auto *value,
                    bool fixed) {
        const auto *attribute = source.Attribute(property);
        if (!attribute)
            return true;
        if (fixed && (attribute->fact.mightBeTimeVarying ||
                      !attribute->fact.sampleTimes.empty()))
            return Fail(error, "time-varying solver structure: " +
                                   property.GetString());
        RigExecSceneBoundInput binding;
        if (!source.Bind(property, RigExecSceneReadRoute::Raw, &binding, error))
            return false;
        VtValue raw;
        if (!source.Read(binding, time, &raw, nullptr, error))
            return false;
        using T = std::decay_t<decltype(*value)>;
        if (raw.IsEmpty())
            return true;
        if (!raw.IsHolding<T>())
            return true;
        *value = raw.UncheckedGet<T>();
        return true;
    };
    auto token = [&](const TfToken &name, const TfToken &fallback,
                     TfToken *value) {
        *value = fallback;
        return read(path.AppendProperty(name), UsdTimeCode::Default(), value,
                    true);
    };
    auto bind = [&](const TfToken &name, auto fallback) {
        const auto property = path.AppendProperty(name);
        if (!source.Attribute(property))
            return true;
        RigExecSceneBoundInput binding;
        if (!source.Bind(property, RigExecSceneReadRoute::ConnectionResolved,
                         &binding, error))
            return false;
        // Empty schema values retain the typed computation fallback.
        for (auto &value : binding.values)
            if (value.IsEmpty())
                value = VtValue(fallback);
        RigExecSceneTypedRead typed;
        if (!RigExecBindSceneTypedRead(
                scene, property, RigExecSceneInputType<decltype(fallback)>(),
                &typed, error))
            return false;
        result.typedReads.emplace(name.GetString(), std::move(typed));
        result.inputs.emplace(name.GetString(), std::move(binding));
        return true;
    };
    auto initial = [&](const char *name, auto fallback) {
        const auto found = result.typedReads.find(name);
        if (found == result.typedReads.end() || scene.identities.empty())
            return fallback;
        VtValue value;
        if (!RigExecResolveSceneTypedRead(scene, found->second,
                                          scene.identities.front(), {}, &value,
                                          nullptr))
            return fallback;
        using T = decltype(fallback);
        return value.IsHolding<T>() ? value.UncheckedGet<T>() : fallback;
    };
    VtIntArray elements;
    const bool consumesElements = s.kind == RigExecSolverKind::TwoBoneIk ||
                                  s.kind == RigExecSolverKind::SplineIk;
    if (consumesElements &&
        !read(path.AppendProperty(_tokens->rigExecJointElements),
              UsdTimeCode::Default(), &elements, true))
        return false;
    for (const auto &joint : targets(_tokens->rigExecJoints))
        if (providerExists(joint))
            result.joints.push_back(joint);
    const bool remap = consumesElements && !elements.empty();
    if (remap && elements.size() != result.joints.size())
        s.degenerate = true;
    for (size_t k = 0; k < result.joints.size(); ++k) {
        result.jointElements.push_back(remap && !s.degenerate ? elements[k]
                                                              : int(k));
        s.jointRests.push_back(rest(result.joints[k]).points);
    }
    s.restIsLive.assign(result.joints.size(), false);
    SdfPath derivedStart;
    if (s.kind == RigExecSolverKind::FkChain ||
        s.kind == RigExecSolverKind::Skeleton2d) {
        // The frame the chain or skeleton hangs from. rigExec:startFramePolicy
        // resolves here, at compile time, into the same declared start-frame
        // input an authored target binds.
        result.start = first(_tokens->rigExecStartFrame, true);
        if (RigExecDeriveSceneStartFrame(scene, path, &derivedStart) !=
                RigExecStartFrameDerivation::Derived ||
            !providerExists(derivedStart))
            derivedStart = SdfPath();
        if (!derivedStart.IsEmpty())
            result.start = derivedStart;
        s.hasStart = !result.start.IsEmpty();
        if (s.hasStart)
            s.startRest = rest(result.start).points;
    }
    if (s.kind == RigExecSolverKind::FkChain) {
        TfToken mode;
        if (!token(_tokens->rigExecControlSpace, TfToken(), &mode))
            return false;
        s.parentRelative = mode == SchemaValueText::parentRelative;
        if (!token(_tokens->rigExecSegmentScale, _tokens->none, &mode))
            return false;
        s.scaleSegments = mode == SchemaValueText::toChild;
        for (const auto &control : targets(_tokens->rigExecControls))
            if (providerExists(control)) {
                result.controls.push_back(control);
                s.controlRests.push_back(rest(control).points);
            }
    } else if (s.kind == RigExecSolverKind::TwoBoneIk) {
        result.root = first(_tokens->rigExecRootControl, true);
        result.end = first(_tokens->rigExecEffectorControl, true);
        result.pole = first(_tokens->rigExecPoleControl, true);
        s.degenerate = s.degenerate || result.root.IsEmpty() ||
                       result.end.IsEmpty() || result.pole.IsEmpty();
        std::array<bool, 3> seen{};
        for (size_t k = 0; k < result.joints.size(); ++k) {
            const int element = result.jointElements[k];
            if (element < 0 || element >= 3)
                s.degenerate = true;
            else {
                s.ikRests[size_t(element)] = s.jointRests[k];
                seen[size_t(element)] = true;
            }
        }
        s.degenerate = s.degenerate || !(seen[0] && seen[1] && seen[2]);
        result.space = first(_tokens->rigExecSpace, true);
        if (!bind(_tokens->rigExecSpaceMatrix, GfMatrix4d(1.0)) ||
            !bind(_tokens->rigExecPreferredBendRadians, 0.0) ||
            !bind(_tokens->rigExecUpperLengthOffset, 0.0) ||
            !bind(_tokens->rigExecLowerLengthOffset, 0.0) ||
            !bind(_tokens->inputsStretch, 1.0f) ||
            !bind(_tokens->inputsSoftness, 0.0f) ||
            !bind(_tokens->inputsPin, 0.0f) ||
            !bind(_tokens->inputsUpperScale, 1.0) ||
            !bind(_tokens->inputsLowerScale, 1.0) ||
            !bind(_tokens->inputsSoftDistance, 0.0f) ||
            !bind(_tokens->inputsTwist, 0.0f))
            return false;
        TfToken policy, scaling;
        if (!token(_tokens->rigExecStretchPolicy, _tokens->uniformSegments,
                   &policy) ||
            !token(_tokens->rigExecSegmentScale, _tokens->none, &scaling) ||
            !read(path.AppendProperty(_tokens->rigExecScaleCalibration),
                  UsdTimeCode::Default(), &s.ikParams.limb.scaleCalibration,
                  true))
            return false;
        s.ikParams.softDistancePolicy = policy == SchemaValueText::softDistance;
        s.ikParams.scaleSegments = scaling == SchemaValueText::toChild;
        s.ikParams.preferredBendRadians =
            initial(SchemaAttrText::rigExecPreferredBendRadians, 0.0);
        s.ikParams.stretch = initial(SchemaAttrText::inputsStretch, 1.0f);
        s.ikParams.softness = initial(SchemaAttrText::inputsSoftness, 0.0f);
        if (!s.degenerate)
            RigExecTwoBoneIkLengths(
                s.ikRests,
                initial(SchemaAttrText::rigExecSpaceMatrix, GfMatrix4d(1.0)),
                initial(SchemaAttrText::rigExecUpperLengthOffset, 0.0),
                initial(SchemaAttrText::rigExecLowerLengthOffset, 0.0),
                &s.ikParams.upperLength, &s.ikParams.lowerLength);

    } else if (s.kind == RigExecSolverKind::BlendPointFrames) {
        result.blendA = first(_tokens->rigExecInputA, false);
        result.blendB = first(_tokens->rigExecInputB, false);
        TfToken mode;
        if (!token(_tokens->rigExecScaleBlend, TfToken(), &mode))
            return false;
        s.scaleMode = mode == SchemaValueText::linear
                          ? RigExecScaleBlend::Linear
                          : RigExecScaleBlend::Log;
        if (!token(_tokens->rigExecRotationBlend, _tokens->shortestArc, &mode))
            return false;
        s.blendRotationRejected = mode != SchemaValueText::shortestArc;
        if (!bind(_tokens->inputsWeight, 0.0f))
            return false;
    } else if (s.kind == RigExecSolverKind::TwistDistribution) {
        result.root = first(_tokens->rigExecStart, true);
        result.end = first(_tokens->rigExecEnd, true);
        s.degenerate =
            s.degenerate || result.root.IsEmpty() || result.end.IsEmpty();
        if (!result.root.IsEmpty())
            s.twistStartRest = rest(result.root).points;
        if (!result.end.IsEmpty())
            s.twistEndRest = rest(result.end).points;
        VtFloatArray weights;
        int count = 1;
        if (!read(path.AppendProperty(_tokens->rigExecWeights),
                  UsdTimeCode::Default(), &weights, true) ||
            !read(path.AppendProperty(_tokens->rigExecCount),
                  UsdTimeCode::Default(), &count, true))
            return false;
        for (float weight : weights)
            s.twistWeights.push_back(weight);
        RigExecResolveTwistWeights(count, &s.twistWeights);
        if (!bind(_tokens->inputsTwistTurns, 0.0))
            return false;
    } else if (s.kind == RigExecSolverKind::Ribbon) {
        result.ribbonPoints = first(_tokens->rigExecDriverCurve, false);
        if (result.ribbonPoints.IsPrimPath())
            result.ribbonPoints =
                result.ribbonPoints.AppendProperty(_tokens->points);
        const auto *relation = source.Relationship(
            path.AppendProperty(_tokens->rigExecDriverCurve));
        result.ribbonReadPhase = relation ? relation->readPhase : _tokens->base;
        VtVec3fArray points;
        if (!result.ribbonPoints.IsEmpty() &&
            !read(result.ribbonPoints, UsdTimeCode::Default(), &points, false))
            return false;
        s.ribbonRestPoints.assign(points.begin(), points.end());
        if (!result.ribbonPoints.IsEmpty() &&
            source.Attribute(result.ribbonPoints)) {
            RigExecSceneBoundInput binding;
            if (!source.Bind(result.ribbonPoints, RigExecSceneReadRoute::Raw,
                             &binding, error))
                return false;
            result.inputs.emplace("driverPoints", std::move(binding));
        }
        if (!bind(_tokens->rigExecSampleCount, 5))
            return false;
    } else if (s.kind == RigExecSolverKind::SplineIk) {
        result.root = first(_tokens->rigExecRootControl, true);
        result.mid = first(_tokens->rigExecMidControl, true);
        result.end = first(_tokens->rigExecEndControl, true);
        result.space = first(_tokens->rigExecSpace, true);
        s.degenerate = s.degenerate || result.root.IsEmpty() ||
                       result.mid.IsEmpty() || result.end.IsEmpty();
        const size_t count = result.joints.size();
        s.splineCount = count;
        s.degenerate = s.degenerate || count == 0;
        s.splineRestFrames.resize(count);
        s.splineJointRests.resize(count);
        std::vector<bool> filled(count, false);
        for (size_t k = 0; k < count; ++k) {
            const int element = result.jointElements[k];
            if (element < 0 || size_t(element) >= count ||
                filled[size_t(element)]) {
                s.degenerate = true;
                continue;
            }
            filled[size_t(element)] = true;
            s.splineRestFrames[size_t(element)] = rest(result.joints[k]);
            s.splineJointRests[size_t(element)] = s.jointRests[k];
        }
        VtFloatArray weights;
        TfToken mode;
        if (!read(path.AppendProperty(_tokens->rigExecVolumeWeights),
                  UsdTimeCode::Default(), &weights, true) ||
            !token(_tokens->rigExecRestLength, TfToken(), &mode))
            return false;
        for (float weight : weights)
            s.splineRestWeights.push_back(weight);
        if (!weights.empty() && weights.size() != count)
            s.degenerate = true;
        if (!mode.IsEmpty() && mode != SchemaValueText::curve &&
            mode != SchemaValueText::chain)
            s.degenerate = true;
        s.splineRestMode = mode == SchemaValueText::chain
                               ? RigExecSplineIkRestLength::Chain
                               : RigExecSplineIkRestLength::Curve;
        if (!result.root.IsEmpty())
            s.splineRootRest = rest(result.root);
        if (!result.mid.IsEmpty())
            s.splineMidRest = rest(result.mid);
        if (!result.end.IsEmpty())
            s.splineEndRest = rest(result.end);
        s.splineRest = RigExecSplineIkMakeRest(
            s.splineRestFrames, s.splineRootRest, s.splineMidRest,
            s.splineEndRest, s.splineRestWeights, s.splineRestMode);
        if (!token(_tokens->rigExecRootTangent, TfToken(), &mode))
            return false;
        s.splineParams.aimRootTangent = mode == SchemaValueText::aim;
        if (!mode.IsEmpty() && mode != SchemaValueText::aim &&
            mode != SchemaValueText::rigid)
            s.degenerate = true;
        if (!bind(_tokens->inputsPreserveVolume, 1.0) ||
            !bind(_tokens->inputsMidFollowWeight, 0.5) ||
            !bind(_tokens->inputsRoll, 0.0) ||
            !bind(_tokens->inputsTwist, 0.0) ||
            !bind(_tokens->inputsMinLengthRatio, 0.0))
            return false;
        s.splineParams.preserveVolume =
            initial(SchemaAttrText::inputsPreserveVolume, 1.0);
        s.splineParams.midFollowWeight =
            initial(SchemaAttrText::inputsMidFollowWeight, 0.5);
        s.splineParams.roll =
            GfDegreesToRadians(initial(SchemaAttrText::inputsRoll, 0.0));
        s.splineParams.twist =
            GfDegreesToRadians(initial(SchemaAttrText::inputsTwist, 0.0));
        s.splineParams.minLengthRatio =
            initial(SchemaAttrText::inputsMinLengthRatio, 0.0);

    } else if (s.kind == RigExecSolverKind::Skeleton2d) {
        RigExecSceneSkeleton2dDescriptor skeleton;
        if (!RigExecTranslateSceneSkeleton2d(scene, path, result.start, rests,
                                             &skeleton, error))
            return false;
        if (skeleton.joints != result.joints)
            return Fail(error,
                        path.GetString() +
                            ": every 2D skeleton bone must be a pose provider");
        s.bone2dOrder = std::move(skeleton.solveOrder);
        s.bone2dSetup = std::move(skeleton.setup);
        s.bone2dSpaceSources = std::move(skeleton.spaceSources);
        s.bone2dTargetCandidates = std::move(skeleton.targetCandidates);
        // Outside parent spaces ride on the shared controls: provider frames
        // read out of the pose exactly as FK controls are.
        for (const auto &space : skeleton.externalSources) {
            result.controls.push_back(space);
            s.controlRests.push_back(rest(space).points);
        }
        result.bone2dChannelReads = std::move(skeleton.channelReads);
        result.bone2dSelectorReads = std::move(skeleton.selectorReads);
        result.bone2dSelectorFallbacks = std::move(skeleton.selectorFallbacks);
    }
    if (s.kind == RigExecSolverKind::FkChain ||
        s.kind == RigExecSolverKind::Skeleton2d) {
        // Members index the joints as the solve does: a 2D bone by its
        // solve order, an FK joint by its rigExec:joints position.
        std::map<SdfPath, int32_t> jointIndex;
        if (s.kind == RigExecSolverKind::Skeleton2d) {
            for (size_t e = 0; e < s.bone2dOrder.size(); ++e)
                jointIndex.emplace(result.joints[s.bone2dOrder[e]], int32_t(e));
        } else {
            for (size_t k = 0; k < result.joints.size(); ++k)
                jointIndex.emplace(result.joints[k], int32_t(k));
        }
        std::string who;
        if (!RigExecSpellPathText(path, &who))
            who = path.GetAsString();
        if (!RigExecTranslateSceneJointGroups(
                scene, path, jointIndex, who, &s.jointGroups,
                &s.jointGroupMembers, &result.jointGroupSelectorReads, error))
            return false;
        if (s.kind == RigExecSolverKind::FkChain && !s.jointGroups.empty()) {
            s.jointGroupParents.assign(result.joints.size(), -1);
            for (size_t k = 0; k < result.joints.size(); ++k) {
                for (SdfPath up = result.joints[k].GetParentPath();
                     !up.IsEmpty() && !up.IsAbsoluteRootPath();
                     up = up.GetParentPath()) {
                    const auto found = jointIndex.find(up);
                    if (found != jointIndex.end()) {
                        s.jointGroupParents[k] = found->second;
                        break;
                    }
                }
            }
        }
    }
    for (const TfToken *name :
         {&_tokens->rigExecControls, &_tokens->rigExecStartFrame,
          &_tokens->rigExecRootControl, &_tokens->rigExecMidControl,
          &_tokens->rigExecEffectorControl, &_tokens->rigExecEndControl,
          &_tokens->rigExecPoleControl, &_tokens->rigExecStart,
          &_tokens->rigExecEnd, &_tokens->rigExecSpace, &_tokens->rigExecInputA,
          &_tokens->rigExecInputB}) {
        const auto relationPath = path.AppendProperty(*name);
        RigExecSceneSolverFrameBinding binding;
        const auto *relation = source.Relationship(relationPath);
        binding.phase = relation ? relation->readPhase : _tokens->base;
        if (relation)
            for (const auto &target : relation->forwardedTargets)
                binding.keys.emplace(target, relationPath.AppendTarget(target));
        // A derived start binds exactly as the authored target would.
        if (name == &_tokens->rigExecStartFrame && !derivedStart.IsEmpty())
            binding.keys.emplace(derivedStart,
                                 relationPath.AppendTarget(derivedStart));
        result.frameBindings.emplace(name->GetString(), std::move(binding));
    }
    if (!result.space.IsEmpty())
        result.spaceRest = rest(result.space).points;
    *output = std::move(result);
    return true;
}
bool
RigExecResolveSceneSolverInputs(
    const RigExecSceneDescriptors &scene,
    const RigExecSceneSolverDescriptor &descriptor, UsdTimeCode identity,
    const std::map<SdfPath, RigExecPointFrame> &frames,
    const std::map<SdfPath, RigExecPointFrameArray> &aggregates,
    const std::map<SdfPath, VtValue> &deliveredInputs,
    RigExecSolverInputs *output, std::string *error)
{
    if (!output)
        return Fail(error, "null solver inputs destination");
    const RigExecSceneCompileInputs source(scene);
    auto &input = *output;
    auto frame = [&](const SdfPath &path, const char *relationName,
                     RigExecPointFrame *value) {
        if (path.IsEmpty())
            return true;
        const auto relation = descriptor.frameBindings.find(relationName);
        if (relation != descriptor.frameBindings.end()) {
            const auto key = relation->second.keys.find(path);
            if (key != relation->second.keys.end()) {
                const auto bound = frames.find(key->second);
                if (bound != frames.end()) {
                    *value = bound->second;
                    return true;
                }
            }
            if (relation->second.phase != "base")
                return Fail(error,
                            "phased solver frame producer unavailable: " +
                                path.GetString());
        }
        const auto found = frames.find(path);
        if (found == frames.end())
            return Fail(error, "solver frame producer unavailable: " +
                                   path.GetString());
        *value = found->second;
        return true;
    };
    const auto readValue = [&](const char *name, VtValue *result) {
        const auto found = descriptor.inputs.find(name);
        *result = VtValue();
        if (found == descriptor.inputs.end())
            return true;
        const auto &binding = found->second;
        VtValue boxed;
        const auto typed = descriptor.typedReads.find(name);
        if (typed != descriptor.typedReads.end()) {
            if (binding.readPhase != "base" &&
                binding.source != binding.consumer &&
                !deliveredInputs.count(binding.consumer))
                return Fail(error,
                            "phased solver input producer unavailable: " +
                                binding.consumer.GetString());
            if (!RigExecResolveSceneTypedRead(scene, typed->second, identity,
                                              deliveredInputs, &boxed, nullptr))
                return true;
        } else {
            const auto delivered = deliveredInputs.find(binding.consumer);
            if (binding.route != RigExecSceneReadRoute::Raw &&
                delivered != deliveredInputs.end())
                boxed = delivered->second;
            else if (!source.Read(binding, identity, &boxed, nullptr, error))
                return false;
        }
        *result = std::move(boxed);
        return true;
    };
    if (descriptor.record.degenerate)
        return true;
    input.controls.resize(descriptor.controls.size());
    for (size_t k = 0; k < descriptor.controls.size(); ++k)
        if (!frame(descriptor.controls[k], SchemaAttrText::rigExecControls,
                   &input.controls[k]))
            return false;
    const bool twist =
        descriptor.record.kind == RigExecSolverKind::TwistDistribution;
    const bool twoBone = descriptor.record.kind == RigExecSolverKind::TwoBoneIk;
    if (!frame(descriptor.start, SchemaAttrText::rigExecStartFrame,
               &input.start) ||
        !frame(descriptor.root,
               twist ? SchemaAttrText::rigExecStart
                     : SchemaAttrText::rigExecRootControl,
               &input.root) ||
        !frame(descriptor.mid, SchemaAttrText::rigExecMidControl, &input.mid) ||
        !frame(descriptor.end,
               twist     ? SchemaAttrText::rigExecEnd
               : twoBone ? SchemaAttrText::rigExecEffectorControl
                         : SchemaAttrText::rigExecEndControl,
               &input.end) ||
        !frame(descriptor.pole, SchemaAttrText::rigExecPoleControl,
               &input.pole) ||
        !frame(descriptor.space, SchemaAttrText::rigExecSpace, &input.space))
        return false;
    input.hasSpace = !descriptor.space.IsEmpty();
    input.spaceRest = descriptor.spaceRest;
    input.blendA = nullptr;
    input.blendB = nullptr;
    auto aggregate = [&](const SdfPath &path, const char *name,
                         const RigExecPointFrameArray **value) {
        if (path.IsEmpty())
            return true;
        const auto relation = descriptor.frameBindings.find(name);
        if (relation != descriptor.frameBindings.end()) {
            const auto key = relation->second.keys.find(path);
            if (key != relation->second.keys.end()) {
                const auto phased = aggregates.find(key->second);
                if (phased != aggregates.end()) {
                    *value = &phased->second;
                    return true;
                }
            }
            if (relation->second.phase != "base")
                return Fail(error,
                            "phased solver aggregate producer unavailable: " +
                                path.GetString());
        }
        const auto found = aggregates.find(path);
        if (found != aggregates.end())
            *value = &found->second;
        return true;
    };
    if (!aggregate(descriptor.blendA, SchemaAttrText::rigExecInputA,
                   &input.blendA) ||
        !aggregate(descriptor.blendB, SchemaAttrText::rigExecInputB,
                   &input.blendB))
        return false;
    if (descriptor.record.kind == RigExecSolverKind::Skeleton2d) {
        const size_t bones =
            descriptor.bone2dChannelReads.size() / kRigExecBone2dChannelCount;
        input.bone2dChannels.assign(bones, RigExecBone2dChannels());
        for (size_t k = 0; k < descriptor.bone2dChannelReads.size(); ++k) {
            VtValue boxed;
            const auto &typed = descriptor.bone2dChannelReads[k];
            if (typed.readPhase != "base" &&
                !deliveredInputs.count(typed.consumer))
                return Fail(error,
                            "phased 2D bone channel producer unavailable: " +
                                typed.consumer.GetString());
            if (!RigExecResolveSceneTypedRead(scene, typed, identity,
                                              deliveredInputs, &boxed, nullptr))
                continue;
            RigExecSetBone2dChannel(
                &input.bone2dChannels[k / kRigExecBone2dChannelCount],
                k % kRigExecBone2dChannelCount, boxed);
        }
        input.bone2dSelectors = descriptor.bone2dSelectorFallbacks;
        input.bone2dSelectors.resize(descriptor.bone2dSelectorReads.size(),
                                     0.0);
        for (size_t g = 0; g < descriptor.bone2dSelectorReads.size(); ++g) {
            VtValue boxed;
            const auto &typed = descriptor.bone2dSelectorReads[g];
            if (typed.readPhase != "base" &&
                !deliveredInputs.count(typed.consumer))
                return Fail(error, "phased 2D selector producer unavailable: " +
                                       typed.consumer.GetString());
            if (!RigExecResolveSceneTypedRead(scene, typed, identity,
                                              deliveredInputs, &boxed, nullptr))
                continue;
            if (boxed.IsHolding<double>())
                input.bone2dSelectors[g] = boxed.UncheckedGet<double>();
        }
    }
    input.jointGroupSelectors.assign(descriptor.jointGroupSelectorReads.size(),
                                     0.0);
    for (size_t g = 0; g < descriptor.jointGroupSelectorReads.size(); ++g) {
        VtValue boxed;
        const auto &typed = descriptor.jointGroupSelectorReads[g];
        if (typed.readPhase != "base" && !deliveredInputs.count(typed.consumer))
            return Fail(error,
                        "phased joint group selector producer unavailable: " +
                            typed.consumer.GetString());
        if (!RigExecResolveSceneTypedRead(scene, typed, identity,
                                          deliveredInputs, &boxed, nullptr))
            continue;
        if (boxed.IsHolding<double>())
            input.jointGroupSelectors[g] = boxed.UncheckedGet<double>();
    }
    return RigExecRefreshSolverParameters(descriptor.record.kind, readValue,
                                          &input, error);
}

bool
RigExecRefreshSolverParameters(
    RigExecSolverKind kind,
    const std::function<bool(const char *, VtValue *)> &read,
    RigExecSolverInputs *output, std::string *error)
{
    if (!output)
        return Fail(error, "null solver parameters destination");
    auto &input = *output;
    const auto value = [&](const char *name, auto fallback, auto *result) {
        *result = fallback;
        VtValue boxed;
        if (!read(name, &boxed))
            return false;
        using T = std::decay_t<decltype(*result)>;
        if (boxed.IsHolding<T>())
            *result = boxed.UncheckedGet<T>();
        return true;
    };
    switch (kind) {
    case RigExecSolverKind::TwoBoneIk:
        input.refreshIkParams = true;
        return value(SchemaAttrText::rigExecSpaceMatrix, GfMatrix4d(1.0),
                     &input.ikSpace) &&
               value(SchemaAttrText::rigExecPreferredBendRadians, 0.0,
                     &input.bend) &&
               value(SchemaAttrText::rigExecUpperLengthOffset, 0.0,
                     &input.upperOffset) &&
               value(SchemaAttrText::rigExecLowerLengthOffset, 0.0,
                     &input.lowerOffset) &&
               value(SchemaAttrText::inputsStretch, 1.0f, &input.stretch) &&
               value(SchemaAttrText::inputsSoftness, 0.0f, &input.softness) &&
               value(SchemaAttrText::inputsPin, 0.0f, &input.pin) &&
               value(SchemaAttrText::inputsUpperScale, 1.0,
                     &input.upperScale) &&
               value(SchemaAttrText::inputsLowerScale, 1.0,
                     &input.lowerScale) &&
               value(SchemaAttrText::inputsSoftDistance, 0.0f,
                     &input.softDistance) &&
               value(SchemaAttrText::inputsTwist, 0.0f, &input.limbTwist);
    case RigExecSolverKind::BlendPointFrames: {
        float weight = 0;
        if (!value(SchemaAttrText::inputsWeight, 0.0f, &weight))
            return false;
        input.blendWeight = double(weight);
        return true;
    }
    case RigExecSolverKind::TwistDistribution:
        return value(SchemaAttrText::inputsTwistTurns, 0.0, &input.twistTurns);
    case RigExecSolverKind::Ribbon: {
        VtVec3fArray points;
        if (!value("driverPoints", VtVec3fArray{}, &points) ||
            !value(SchemaAttrText::rigExecSampleCount, 5,
                   &input.ribbonSampleCount))
            return false;
        input.ribbonPoints.assign(points.begin(), points.end());
        return true;
    }
    case RigExecSolverKind::SplineIk:
        input.refreshSplineParams = true;
        return value(SchemaAttrText::inputsPreserveVolume, 1.0,
                     &input.preserveVolume) &&
               value(SchemaAttrText::inputsMidFollowWeight, 0.5,
                     &input.midFollowWeight) &&
               value(SchemaAttrText::inputsRoll, 0.0, &input.rollDegrees) &&
               value(SchemaAttrText::inputsTwist, 0.0, &input.twistDegrees) &&
               value(SchemaAttrText::inputsMinLengthRatio, 0.0,
                     &input.minLengthRatio);
    case RigExecSolverKind::FkChain:
        return true;
    // Its channels are per-bone typed reads, filled by the caller.
    case RigExecSolverKind::Skeleton2d:
        return true;
    }
    return Fail(error, "unknown normalized solver input kind");
}

} // namespace rigExec
