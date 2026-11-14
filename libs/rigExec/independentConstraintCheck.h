#ifndef RIGEXEC_INDEPENDENT_CONSTRAINT_CHECK_H
#define RIGEXEC_INDEPENDENT_CONSTRAINT_CHECK_H

#include "independentConstraintReference.h"
#include "bakedExecCrossCheck.h"
#include "goldenPose.h"
#include "moverGraphCaches.h"
#include "frameExtraction.h"
#include "rigEvaluator.h"
#include "rigExecSchema/schemaNames.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/vt/array.h"
#include <algorithm>

namespace rigExec {
namespace independentConstraintCheck {

TF_DEFINE_PRIVATE_TOKENS(_constraintCheckTokens,
    (computePointFrame)
    (computeBasePointFrame)
    ((rigExecSources, SchemaAttrText::rigExecSources))
    ((rigExecAimTarget, SchemaAttrText::rigExecAimTarget))
    ((rigExecMoves, SchemaAttrText::rigExecMoves))
    ((rigExecWeightObject, SchemaAttrText::rigExecWeightObject))
    ((rigExecRotationOrder, SchemaAttrText::rigExecRotationOrder))
    ((rigExecBlendShear, SchemaAttrText::rigExecBlendShear))
    ((rigExecAimAxis, SchemaAttrText::rigExecAimAxis))
    ((rigExecWorldUpType, SchemaAttrText::rigExecWorldUpType))
    ((rigExecWorldUpRotationOnly, SchemaAttrText::rigExecWorldUpRotationOnly))
    ((inputsEnabled, SchemaAttrText::inputsEnabled))
    ((inputsDefaultWeight, SchemaAttrText::inputsDefaultWeight))
    ((inputsSourceWeights, SchemaAttrText::inputsSourceWeights))
    ((inputsTranslationOffsets, SchemaAttrText::inputsTranslationOffsets))
    ((inputsRotationOffsets, SchemaAttrText::inputsRotationOffsets))
    ((inputsAffectTranslationX, SchemaAttrText::inputsAffectTranslationX))
    ((inputsAffectTranslationY, SchemaAttrText::inputsAffectTranslationY))
    ((inputsAffectTranslationZ, SchemaAttrText::inputsAffectTranslationZ))
    ((inputsAffectRotationX, SchemaAttrText::inputsAffectRotationX))
    ((inputsAffectRotationY, SchemaAttrText::inputsAffectRotationY))
    ((inputsAffectRotationZ, SchemaAttrText::inputsAffectRotationZ))
    ((inputsAffectScaleX, SchemaAttrText::inputsAffectScaleX))
    ((inputsAffectScaleY, SchemaAttrText::inputsAffectScaleY))
    ((inputsAffectScaleZ, SchemaAttrText::inputsAffectScaleZ))
    ((inputsRotationOffset, SchemaAttrText::inputsRotationOffset))
    ((inputsAimVector, SchemaAttrText::inputsAimVector))
    ((inputsUpVector, SchemaAttrText::inputsUpVector))
    ((inputsWorldUpVector, SchemaAttrText::inputsWorldUpVector))
    ((posedDefaultSpace, SchemaAttrText::posedDefaultSpace))
    ((posedSpace, SchemaAttrText::posedSpace))
    ((xyz, SchemaValueText::xyz))
    (x)
    (none)
);

struct Source {
    SdfPath path;
    bool boundFrame = false;
    std::vector<SdfPath> ancestors;
};
struct Binding {
    SdfPath owner, target, assetRoot, carry;
    TfToken type;
    std::vector<Source> sources;
    Source worldUp;
    bool geometry = false;
};

inline const VtValue *
Find(const std::vector<RigExecValueOverride> &inputs, const SdfPath &path,
     const TfToken &computation)
{
    for (const auto &input : inputs)
        if (input.prim == path && input.attribute.IsEmpty() &&
            input.computation == computation)
            return &input.value;
    return nullptr;
}
template <class T>
inline bool
Bound(const std::vector<RigExecValueOverride> &inputs, const SdfPath &path,
      const TfToken &computation, T *out)
{
    const auto *value = Find(inputs, path, computation);
    if (!value || !value->IsHolding<T>())
        return false;
    *out = value->UncheckedGet<T>();
    return true;
}
template <class T>
inline T
Read(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
     const TfToken &name, T fallback, UsdTimeCode time)
{
    T value = fallback;
    resolved.GetAttribute(prim.GetAttribute(name), time, &value);
    return value;
}
template <class T>
inline T
Raw(const UsdPrim &prim, const TfToken &name, T fallback,
    UsdTimeCode time = UsdTimeCode::Default())
{
    T value = fallback;
    const auto attr = prim.GetAttribute(name);
    if (attr)
        attr.Get(&value, time);
    return value;
}
inline bool
Usable(const RigExecPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate())
        return false;
    for (const auto &point : frame.points)
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(point[axis]))
                return false;
    return true;
}
inline bool
StageFrame(const UsdStageRefPtr &stage, const UsdPrim &asset,
           UsdGeomXformCache *cache, const SdfPath &path,
           RigExecPointFrame *frame, GfMatrix4d *matrix = nullptr)
{
    const auto prim = stage->GetPrimAtPath(path);
    if (!prim || !asset || !UsdGeomXformable(prim))
        return false;
    bool resets = false;
    auto relative = cache->ComputeRelativeTransform(prim, asset, &resets);
    if (resets)
        relative = cache->GetLocalToWorldTransform(prim) *
                   cache->GetLocalToWorldTransform(asset).GetInverse();
    if (frame)
        *frame = RigExecFrameFromMatrix(relative);
    if (matrix)
        *matrix = relative;
    return true;
}
inline bool
ResolveSource(const Binding &binding, const Source &source,
              const UsdStageRefPtr &stage, UsdGeomXformCache *cache,
              const std::vector<RigExecValueOverride> &inputs,
              RigExecPointFrame *frame)
{
    if (source.boundFrame)
        return Bound(inputs, source.path,
                     _constraintCheckTokens->computePointFrame, frame) &&
               frame->IsValid();
    if (!StageFrame(stage, stage->GetPrimAtPath(binding.assetRoot), cache,
                    source.path, frame))
        return false;
    bool complete = true;
    const auto enumerate = [&](const RigExecPoseFrameVisitor &visit) {
        for (const auto &path : source.ancestors) {
            RigExecPointFrame base, current;
            if (!Bound(inputs, path,
                       _constraintCheckTokens->computeBasePointFrame, &base) ||
                !Bound(inputs, path, _constraintCheckTokens->computePointFrame,
                       &current)) {
                complete = false;
                continue;
            }
            visit(path, base, current);
        }
    };
    const RigExecPoseFrameEnumerator providers(enumerate);
    const bool resolved =
        RigExecApplyRevisedAncestorDelta(source.path, providers, frame);
    return complete && resolved;
}
inline RigExecEulerOrder
Order(const TfToken &token)
{
    if (token == "XZY")
        return RigExecEulerOrder::XZY;
    if (token == "YXZ")
        return RigExecEulerOrder::YXZ;
    if (token == "YZX")
        return RigExecEulerOrder::YZX;
    if (token == "ZXY")
        return RigExecEulerOrder::ZXY;
    if (token == "ZYX")
        return RigExecEulerOrder::ZYX;
    return RigExecEulerOrder::XYZ;
}
inline RigExecConstraintAxisMask
Mask(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
     const TfToken &x, const TfToken &y, const TfToken &z, bool fallback,
     UsdTimeCode time)
{
    RigExecConstraintAxisMask mask;
    mask.x = Read(resolved, prim, x, fallback, time);
    mask.y = Read(resolved, prim, y, fallback, time);
    mask.z = Read(resolved, prim, z, fallback, time);
    return mask;
}
inline bool
Encode(bool present, bool abandoned, bool geometry, bool deltaPresent,
       const RigExecPointFrame *candidate, const RigExecPointFrame *published,
       const GfMatrix4d *delta, VtValue *out, std::string *error = nullptr)
{
    std::string text = "present=" + std::to_string(present) +
                       ";abandoned=" + std::to_string(abandoned) +
                       ";geometry=" + std::to_string(geometry) +
                       ";deltaPresent=" + std::to_string(deltaPresent);
    const auto append = [&](const char *name, const VtValue &value) {
        std::string bits;
        if (!RigExecEncodeGoldenValue(value, &bits, error))
            return false;
        text += std::string(";") + name + "=" + bits;
        return true;
    };
    if (candidate && !append("candidate", VtValue(*candidate)))
        return false;
    if (published && !append("published", VtValue(*published)))
        return false;
    if (delta && !append("delta", VtValue(*delta)))
        return false;
    *out = VtValue(text);
    return true;
}

struct Prepared {
    RigExecIndependentConstraintInputs packet;
    RigExecPointFrame entering;
    GfMatrix4d carry{1.0}, base{1.0};
    bool geometry = false, hasCarry = false, admitted = false, dormant = false,
         baseAvailable = false;
};

// Owning-thread context acquisition. Bound frames are pre-body copies; raw
// tables and schema contracts are read independently from the ORIGINAL stage.
inline bool
Prepare(const Binding &binding, const UsdStageRefPtr &stage, UsdTimeCode time,
        const std::vector<RigExecValueOverride> &inputs, Prepared *out,
        std::string *error)
{
    const auto fail = [&](const char *message) {
        if (error)
            *error = message;
        return false;
    };
    if (!stage || !out)
        return fail("missing independent constraint stage/output");
    const auto prim = stage->GetPrimAtPath(binding.owner);
    if (!prim || prim.GetTypeName() != binding.type)
        return fail("independent constraint owner/type changed");
    SdfPathVector sources, targets;
    if (const auto rel =
            prim.GetRelationship(_constraintCheckTokens->rigExecSources))
        rel.GetForwardedTargets(&sources);
    if (sources.empty() && binding.type == SchemaTypeName::RigExecAimConstraint)
        if (const auto rel =
                prim.GetRelationship(_constraintCheckTokens->rigExecAimTarget))
            rel.GetForwardedTargets(&sources);
    if (const auto rel =
            prim.GetRelationship(_constraintCheckTokens->rigExecMoves))
        rel.GetForwardedTargets(&targets);
    if (targets.size() != 1 || targets[0].GetPrimPath() != binding.target ||
        (targets[0].IsPropertyPath() &&
         targets[0].GetNameToken() == "points") != binding.geometry ||
        sources.size() != binding.sources.size())
        return fail("independent constraint relationship context changed");
    for (size_t i = 0; i < sources.size(); ++i)
        if (sources[i].GetPrimPath() != binding.sources[i].path)
            return fail(
                "independent source identity differs from captured input");
    RigExecResolvedInputs resolved;
    for (const auto &input : inputs)
        if (!input.attribute.IsEmpty())
            resolved.SetProperty(input.prim.AppendProperty(input.attribute),
                                 input.value);
    RigExecPointFrame entering;
    if (!Bound(inputs, binding.target,
               _constraintCheckTokens->computePointFrame, &entering))
        return fail("missing independent entering frame");
    *out = Prepared();
    out->entering = entering;
    out->geometry = binding.geometry;
    const auto finish = [&] { return true; };
    if (!Read(resolved, prim, _constraintCheckTokens->inputsEnabled, true,
              time))
        return finish();
    const SdfPathVector weightTargets = [&] {
        SdfPathVector targets;
        if (const auto rel = prim.GetRelationship(
                _constraintCheckTokens->rigExecWeightObject))
            rel.GetForwardedTargets(&targets);
        return targets;
    }();
    if (!weightTargets.empty() && !binding.geometry)
        return fail("independent scalar weight-object witness is unavailable");
    double weight = 1.0;
    if (weightTargets.empty()) {
        weight = Read(resolved, prim,
                      _constraintCheckTokens->inputsDefaultWeight, 1.0f, time);
        if (!std::isfinite(weight) || weight < 0 || weight > 1)
            return finish();
    }
    if (weight <= 0 && (!binding.geometry || weightTargets.empty())) {
        if (binding.geometry)
            out->dormant = true;
        return finish();
    }
    UsdGeomXformCache cache(time);
    VtFloatArray weights =
        Raw(prim, _constraintCheckTokens->inputsSourceWeights, VtFloatArray(),
            time);
    if (!weights.empty() && weights.size() != binding.sources.size())
        return finish();
    VtVec3dArray translations, rotations;
    if (binding.type == SchemaTypeName::RigExecParentConstraint) {
        translations =
            Raw(prim, _constraintCheckTokens->inputsTranslationOffsets,
                VtVec3dArray(), time);
        rotations = Raw(prim, _constraintCheckTokens->inputsRotationOffsets,
                        VtVec3dArray(), time);
        if ((!translations.empty() &&
             translations.size() != binding.sources.size()) ||
            (!rotations.empty() && rotations.size() != binding.sources.size()))
            return finish();
    }
    auto &packet = out->packet;
    packet.entering = entering;
    for (size_t i = 0; i < binding.sources.size(); ++i) {
        RigExecConstraintSource source;
        if (!ResolveSource(binding, binding.sources[i], stage, &cache, inputs,
                           &source.frame))
            return finish();
        source.normalizedWeight = weights.empty() ? 1.0 : weights[i];
        source.translationOffset =
            translations.empty() ? GfVec3d(0) : translations[i];
        source.rotationOffsetDegrees =
            rotations.empty() ? GfVec3d(0) : rotations[i];
        packet.sources.push_back(source);
    }
    const auto order =
        Order(Raw(prim, _constraintCheckTokens->rigExecRotationOrder,
                  _constraintCheckTokens->xyz));
    const auto affect =
        Mask(resolved, prim, _constraintCheckTokens->inputsAffectRotationX,
             _constraintCheckTokens->inputsAffectRotationY,
             _constraintCheckTokens->inputsAffectRotationZ, true, time);
    const double solveWeight = binding.geometry ? 1.0 : weight;
    auto &carry = out->carry;
    out->hasCarry = !binding.carry.IsEmpty();
    if (out->hasCarry) {
        GfMatrix4d d(1.0), p(1.0);
        if (!resolved.Get(binding.carry.AppendProperty(
                              _constraintCheckTokens->posedDefaultSpace),
                          &d) ||
            !resolved.Get(binding.carry.AppendProperty(
                              _constraintCheckTokens->posedSpace),
                          &p))
            out->hasCarry = false;
        else {
            GfMatrix4d check(1.0);
            const auto defaultFrame = RigExecFrameFromMatrix(d),
                       posedFrame = RigExecFrameFromMatrix(p);
            out->hasCarry =
                Usable(defaultFrame) && Usable(posedFrame) &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      defaultFrame.points, &check) &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      posedFrame.points, &check);
            if (out->hasCarry)
                carry = d.GetInverse() * p;
        }
    }
    if (binding.type == SchemaTypeName::RigExecRotationConstraint) {
        packet.kind = RigExecIndependentConstraintInputs::Kind::Rotation;
        packet.rotation.offsetDegrees =
            Read(resolved, prim, _constraintCheckTokens->inputsRotationOffset,
                 GfVec3d(0), time);
        packet.rotation.affect = affect;
        packet.rotation.rotationOrder = order;
        packet.rotation.weight = solveWeight;
        packet.rotation.carry = nullptr;
    } else if (binding.type == SchemaTypeName::RigExecParentConstraint) {
        packet.kind = RigExecIndependentConstraintInputs::Kind::Parent;
        packet.parent.translationAxes = Mask(
            resolved, prim, _constraintCheckTokens->inputsAffectTranslationX,
            _constraintCheckTokens->inputsAffectTranslationY,
            _constraintCheckTokens->inputsAffectTranslationZ, true, time);
        packet.parent.rotationAxes = affect;
        packet.parent.scaleAxes =
            Mask(resolved, prim, _constraintCheckTokens->inputsAffectScaleX,
                 _constraintCheckTokens->inputsAffectScaleY,
                 _constraintCheckTokens->inputsAffectScaleZ, false, time);
        packet.parent.rotationOrder = order;
        packet.parent.weight = solveWeight;
        packet.parent.carry = nullptr;
        packet.parent.blendShear =
            Raw(prim, _constraintCheckTokens->rigExecBlendShear, false);
    } else if (binding.type == SchemaTypeName::RigExecAimConstraint) {
        packet.kind = RigExecIndependentConstraintInputs::Kind::Aim;
        auto &aim = packet.aim;
        aim.localAimVector =
            Read(resolved, prim, _constraintCheckTokens->inputsAimVector,
                 GfVec3d(1, 0, 0), time);
        const auto aimAttr =
            prim.GetAttribute(_constraintCheckTokens->inputsAimVector);
        if (!aimAttr || !aimAttr.HasAuthoredValueOpinion()) {
            const auto axis = Raw(prim, _constraintCheckTokens->rigExecAimAxis,
                                  _constraintCheckTokens->x);
            aim.localAimVector = axis == SchemaValueText::y ? GfVec3d(0, 1, 0)
                                 : axis == SchemaValueText::z
                                     ? GfVec3d(0, 0, 1)
                                     : GfVec3d(1, 0, 0);
        }
        aim.localUpVector =
            Read(resolved, prim, _constraintCheckTokens->inputsUpVector,
                 GfVec3d(0, 1, 0), time);
        aim.rotationOffsetDegrees =
            Read(resolved, prim, _constraintCheckTokens->inputsRotationOffset,
                 GfVec3d(0), time);
        aim.affectRotation = affect;
        aim.rotationOrder = order;
        aim.weight = solveWeight;
        SdfPathVector authoredSources;
        if (const auto rel =
                prim.GetRelationship(_constraintCheckTokens->rigExecSources))
            rel.GetTargets(&authoredSources);
        aim.preserveInputUp = authoredSources.empty();
        double total = 0;
        for (const auto &source : packet.sources) {
            if (!std::isfinite(source.normalizedWeight) ||
                source.normalizedWeight < 0)
                return finish();
            total += source.normalizedWeight;
        }
        const TfToken mode =
            total > 0 ? Raw(prim, _constraintCheckTokens->rigExecWorldUpType,
                            _constraintCheckTokens->none)
                      : _constraintCheckTokens->none;
        const auto up =
            Read(resolved, prim, _constraintCheckTokens->inputsWorldUpVector,
                 GfVec3d(0, 1, 0), time);
        if (mode == "sceneUp") {
            const auto axis = UsdGeomGetStageUpAxis(stage).GetString();
            aim.worldUpDirection = axis == "Z" || axis == SchemaValueText::z
                                       ? GfVec3d(0, 0, 1)
                                       : GfVec3d(0, 1, 0);
        } else if (mode == "vector")
            aim.worldUpDirection = up;
        else if (mode == "objectUp" || mode == "objectRotationUp") {
            RigExecPointFrame object;
            if (binding.worldUp.path.IsEmpty())
                aim.worldUpDirection =
                    mode == "objectUp" ? -entering.Origin() : up;
            else if (!ResolveSource(binding, binding.worldUp, stage, &cache,
                                    inputs, &object))
                return finish();
            else if (mode == "objectUp")
                aim.worldUpDirection = object.Origin() - entering.Origin();
            else {
                GfMatrix4d matrix(1.0);
                if (!RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                           object.points, &matrix))
                    return finish();
                aim.worldUpDirection =
                    (Raw(prim,
                         _constraintCheckTokens->rigExecWorldUpRotationOnly,
                         false)
                         ? matrix.GetOrthonormalized(false)
                         : matrix)
                        .ExtractRotation()
                        .TransformDir(up);
            }
        }
    } else
        return fail("unsupported independent constraint kind");
    out->admitted = true;
    if (binding.geometry)
        out->baseAvailable =
            StageFrame(stage, stage->GetPrimAtPath(binding.assetRoot), &cache,
                       binding.target, nullptr, &out->base);
    return true;
}

// The numerical witness is stage-free and owns every value it consumes.
inline bool
Run(const Prepared &prepared, VtValue *out, std::string *error)
{
    auto packet = prepared.packet;
    GfMatrix4d carry = prepared.carry;
    packet.rotation.carry = prepared.hasCarry ? &carry : nullptr;
    packet.parent.carry = prepared.hasCarry ? &carry : nullptr;
    bool present = false, abandoned = true, deltaPresent = prepared.dormant,
         candidateProduced = false;
    RigExecPointFrame candidate = prepared.entering,
                      published = prepared.entering;
    GfMatrix4d delta(1.0);
    if (prepared.admitted &&
        RigExecIndependentConstraintCandidate(packet, &candidate)) {
        candidateProduced = true;
        if (prepared.geometry) {
            GfMatrix4d solved(1.0);
            if (Usable(candidate) && prepared.baseAvailable &&
                std::isfinite(prepared.base.GetDeterminant()) &&
                prepared.base.GetDeterminant() != 0 &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      candidate.points, &solved)) {
                delta = solved * prepared.base.GetInverse();
                deltaPresent = true;
            }
        } else if (Usable(candidate)) {
            present = true;
            abandoned = false;
            published = candidate;
        }
    }
    return Encode(present, abandoned, prepared.geometry, deltaPresent,
                  candidateProduced ? &candidate : nullptr,
                  prepared.geometry ? nullptr : &published,
                  deltaPresent ? &delta : nullptr, out, error);
}
} // namespace independentConstraintCheck
} // namespace rigExec
#endif
