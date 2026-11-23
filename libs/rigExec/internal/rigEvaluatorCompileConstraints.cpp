// Compile step: pose constraints and the frame chains they revise.

#include "rigExecSchema/schemaNames.h"
#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/internal/rigEvaluatorCompileAttempt.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/inputReplay.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExecScene/poseSceneTranslation.h"
#include "rigExecScene/startFrameDerivation.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/rigEvaluatorDependencies.h"
#include "rigExec/internal/rigEvaluatorPropertyBindings.h"
#include "rigExec/internal/rigEvaluatorConstraints.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/solverKernels.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/pyLock.h"
#include "pxr/base/tf/scoped.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <functional>
#include <limits>
#include <optional>
#include <set>
#include <unordered_map>

namespace rigExec {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((rigExecSpace, SchemaAttrText::rigExecSpace))
    ((rigExecWeightObject, SchemaAttrText::rigExecWeightObject))
    (linear)
    ((rigExecWeightBlend, SchemaAttrText::rigExecWeightBlend))
    ((rigExecTransform, SchemaAttrText::rigExecTransform))
    ((rigExecTransformSpace, SchemaAttrText::rigExecTransformSpace))
    ((rigExecSources, SchemaAttrText::rigExecSources))
    ((rigExecAimTarget, SchemaAttrText::rigExecAimTarget))
    ((rigExecBlendShear, SchemaAttrText::rigExecBlendShear))
    ((rigExecWorldUpRotationOnly, SchemaAttrText::rigExecWorldUpRotationOnly))
    ((rigExecWorldUpObject, SchemaAttrText::rigExecWorldUpObject))
    ((rigExecIkControl, SchemaAttrText::rigExecIkControl))
    ((rigExecPoleControl, SchemaAttrText::rigExecPoleControl))
    ((rigExecRootControl, SchemaAttrText::rigExecRootControl))
    ((rigExecEffector, SchemaAttrText::rigExecEffector))
    ((rigExecFkControls, SchemaAttrText::rigExecFkControls))
    ((rigExecLimbJoints, SchemaAttrText::rigExecLimbJoints))
    ((rigExecFirstJoint, SchemaAttrText::rigExecFirstJoint))
    ((rigExecEndJoint, SchemaAttrText::rigExecEndJoint))
    (rotatePlane)
    ((rigExecSolverMode, SchemaAttrText::rigExecSolverMode))
    ((rigExecPoleVectorObjects, SchemaAttrText::rigExecPoleVectorObjects))
    ((inputsAffectTranslationX, SchemaAttrText::inputsAffectTranslationX))
    ((inputsAffectTranslationY, SchemaAttrText::inputsAffectTranslationY))
    ((inputsAffectTranslationZ, SchemaAttrText::inputsAffectTranslationZ))
    ((inputsAffectRotationX, SchemaAttrText::inputsAffectRotationX))
    ((inputsAffectRotationY, SchemaAttrText::inputsAffectRotationY))
    ((inputsAffectRotationZ, SchemaAttrText::inputsAffectRotationZ))
    ((inputsAffectScaleX, SchemaAttrText::inputsAffectScaleX))
    ((inputsAffectScaleY, SchemaAttrText::inputsAffectScaleY))
    ((inputsAffectScaleZ, SchemaAttrText::inputsAffectScaleZ))
);

using namespace evaluatorDetail;

bool
RigExecRigEvaluator::_CompilePoseConstraints(_CompileAttempt &attempt)
{
    std::vector<std::string> *errors = attempt.errors;
    auto &newJointPaths = attempt.newJointPaths;
    auto &newMovers = attempt.newMovers;
    const std::vector<UsdPrim> &orderedSolvers = attempt.aggregateSolvers;
    auto &newJointBinding = attempt.newJointBinding;
    auto &jointWritingSolvers = attempt.jointWritingSolvers;
    // Pose-domain constraints, compiled to in-memory structural wiring. Aim,
    // Position, Rotation, Scale, and Parent revise one transform provider;
    // SingleChainIK revises its inferred joint chain atomically. Values stay
    // authored on the mover and are sampled during Evaluate().
    auto &newFrameConstraints = attempt.newFrameConstraints;
    auto &newFrameChains = attempt.newFrameChains;

    auto bindFrameSource = [&](const SdfPath &operation,
                               const SdfPath &authored, const std::string &role,
                               _FrameSourceBinding *binding) {
        if (!authored.IsPrimPath()) {
            return attempt.Fail(role + " must target a prim, got " +
                                    authored.GetString(),
                                {operation});
        }
        const UsdPrim sourcePrim = _stage->GetPrimAtPath(authored);
        if (!sourcePrim) {
            return attempt.Fail(role + " targets missing prim " +
                                    authored.GetString(),
                                {operation});
        }
        binding->sourcePath = authored;
        const TfToken sourceType = sourcePrim.GetTypeName();
        if (sourceType == SchemaTypeName::RigExecControl ||
            sourceType == SchemaTypeName::RigExecJoint) {
            return true;
        }
        if (UsdGeomXformable(sourcePrim)) {
            binding->xformPath = authored;
            return true;
        }
        return attempt.Fail(
            role + " targets " + authored.GetString() +
                ", which is neither a RigExec transform provider nor "
                "a UsdGeomXformable",
            {operation});
    };

    // Every property-domain target a math mover revises. A mask attribute in
    // this set is not static: its live read must still see the chain's
    // result, so a constraint whose mask lands here keeps masksStatic false.
    std::set<SdfPath> propertyRevisedTargets;
    for (const RigExecMoverRecord &mover : newMovers) {
        if (!RigExecIsPropertyMover(mover.schemaType)) {
            continue;
        }
        for (const SdfPath &target : mover.targets) {
            propertyRevisedTargets.insert(target);
        }
    }
    for (const RigExecMoverRecord &mover : newMovers) {
        // A matrix mover in the TRANSFORM domain is frame work. It deforms a
        // frame, and a frame is what the pose walk orders, what later movers
        // observe, and what gets published -- none of which the point graph
        // does. Its geometry-domain twin is unaffected and stays there.
        const bool transformDomainMatrix =
            mover.schemaType == SchemaTypeName::RigExecMatrixMover &&
            mover.targets.size() == 1 && mover.targets[0].IsPrimPath() &&
            !rigExec::RigExecIsTransformDomainAmbiguous(_stage,
                                                        mover.targets[0]);
        if (!_IsFrameConstraintType(mover.schemaType) &&
            !transformDomainMatrix) {
            continue;
        }
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        _FrameConstraint constraint;
        constraint.moverPath = mover.moverPath;
        constraint.schemaType = mover.schemaType;
        const SdfPathVector commonWeights = _CompileRelationshipTargets(
            moverPrim, _tokens->rigExecWeightObject);
        if (!commonWeights.empty()) {
            // Phase-A validation already established at-most-one and the
            // exact domain: the moved prim for a source constraint, or this
            // mover prim for an atomic multi-target SingleChainIK.
            constraint.weightObject = commonWeights[0];
        }

        if (transformDomainMatrix) {
            // T = M(transform) * inverse(M(transformSpace)), the same pair
            // the geometry domain reads, carried here as ordered sources so
            // the existing binding and phase machinery resolves them.
            constraint.targets = mover.targets;
            {
                TfToken blend = _tokens->linear;
                if (const UsdAttribute a =
                        moverPrim.GetAttribute(_tokens->rigExecWeightBlend)) {
                    a.Get(&blend);
                }
                constraint.radialBlend = blend == "radial";
            }
            const SdfPathVector xf = _CompileRelationshipTargets(
                moverPrim, _tokens->rigExecTransform);
            if (xf.size() != 1) {
                return attempt.Fail(
                    mover.moverPath.GetString() +
                        ": a transform-domain MatrixMover needs exactly "
                        "one rigExec:transform",
                    {mover.moverPath});
            }
            const SdfPathVector space = _CompileRelationshipTargets(
                moverPrim, _tokens->rigExecTransformSpace);
            for (const SdfPath &one : {xf[0]}) {
                _FrameSourceBinding binding;
                if (!bindFrameSource(mover.moverPath, one,
                                     mover.moverPath.GetString() +
                                         " rigExec:transform",
                                     &binding)) {
                    return false;
                }
                constraint.sources.push_back(binding);
            }
            for (const SdfPath &one : space) {
                _FrameSourceBinding binding;
                if (!bindFrameSource(mover.moverPath, one,
                                     mover.moverPath.GetString() +
                                         " rigExec:transformSpace",
                                     &binding)) {
                    return false;
                }
                constraint.sources.push_back(binding);
                break;
            }
        } else if (_IsSourceFrameConstraintType(mover.schemaType)) {
            constraint.targets = mover.targets;

            // Domain selection, by the authored spelling alone. A bare prim
            // path is the transform domain; <prim>.points is the geometry
            // domain. Either way the frame key is the PRIM -- the solve is
            // identical and only the publish differs -- so the points
            // property is carried aside and targets[0] is normalized.
            if (!constraint.targets.empty() &&
                constraint.targets[0].IsPropertyPath() &&
                constraint.targets[0].GetNameToken() == "points") {
                constraint.pointsTarget = constraint.targets[0];
                constraint.targets[0] = constraint.targets[0].GetPrimPath();
            }

            SdfPathVector sources =
                _CompileRelationshipTargets(moverPrim, _tokens->rigExecSources);
            if (sources.empty() &&
                mover.schemaType == SchemaTypeName::RigExecAimConstraint) {
                // Single-source spelling; sources list is the ordered form.
                sources = _CompileRelationshipTargets(
                    moverPrim, _tokens->rigExecAimTarget);
            }
            if (sources.empty()) {
                return attempt.Fail(mover.schemaType.GetString() + " " +
                                        mover.moverPath.GetString() +
                                        " has no constraint sources",
                                    {mover.moverPath});
            }
            for (const SdfPath &source : sources) {
                _FrameSourceBinding binding;
                if (!bindFrameSource(mover.moverPath, source,
                                     mover.schemaType.GetString() + " " +
                                         mover.moverPath.GetString() +
                                         " source",
                                     &binding)) {
                    return false;
                }
                constraint.sources.push_back(binding);
            }

            // The two operators that read Euler components of their
            // sources -- a mask on one, a weighted mean over several --
            // and so the two whose answer turns with an outer rotation.
            if (mover.schemaType == SchemaTypeName::RigExecRotationConstraint ||
                mover.schemaType == SchemaTypeName::RigExecParentConstraint) {
                const SdfPathVector space = _CompileRelationshipTargets(
                    moverPrim, _tokens->rigExecSpace);
                if (space.size() > 1) {
                    return attempt.Fail(mover.schemaType.GetString() + " " +
                                            mover.moverPath.GetString() +
                                            " has more than one rigExec:space",
                                        {mover.moverPath});
                }
                // A space that is not a frame provider is dropped rather
                // than reported, exactly as the space switch drops it: "no
                // space" and "a space that cannot supply a frame" mean the
                // same thing here, and both leave the mask running as it
                // did before the masters.
                if (!space.empty()) {
                    const UsdPrim spacePrim = _stage->GetPrimAtPath(space[0]);
                    const TfToken spaceType =
                        spacePrim ? spacePrim.GetTypeName() : TfToken();
                    if (spaceType == SchemaTypeName::RigExecControl ||
                        spaceType == SchemaTypeName::RigExecJoint) {
                        constraint.spacePath = space[0];
                    }
                }
            }

            // Opt-in behaviours that change an operator's arithmetic. Both
            // are uniform, so they are compiled once and hashed by the digest.
            const auto readUniformBool = [&moverPrim](const TfToken &name) {
                bool value = false;
                if (const UsdAttribute a = moverPrim.GetAttribute(name)) {
                    a.Get(&value);
                }
                return value;
            };
            if (mover.schemaType == SchemaTypeName::RigExecScaleConstraint ||
                mover.schemaType == SchemaTypeName::RigExecParentConstraint) {
                constraint.blendShear =
                    readUniformBool(_tokens->rigExecBlendShear);
            }
            if (mover.schemaType == SchemaTypeName::RigExecAimConstraint) {
                constraint.worldUpRotationOnly =
                    readUniformBool(_tokens->rigExecWorldUpRotationOnly);
            }

            if (mover.schemaType == SchemaTypeName::RigExecAimConstraint) {
                const SdfPathVector upObjects = _CompileRelationshipTargets(
                    moverPrim, _tokens->rigExecWorldUpObject);
                if (upObjects.size() > 1) {
                    return attempt.Fail(
                        "RigExecAimConstraint " + mover.moverPath.GetString() +
                            " has more than one world-up object",
                        {mover.moverPath});
                }
                if (!upObjects.empty() &&
                    !bindFrameSource(mover.moverPath, upObjects[0],
                                     "RigExecAimConstraint " +
                                         mover.moverPath.GetString() +
                                         " world-up object",
                                     &constraint.worldUpObject)) {
                    return false;
                }
            }
        } else if (mover.schemaType ==
                   SchemaTypeName::RigExecHarmonicsConstraint) {
            // One target, measured and revised; no sources. The optional
            // space measures the motion, and a space that cannot supply a
            // frame is dropped as the rotation constraint drops one.
            constraint.targets = mover.targets;
            const SdfPathVector space =
                _CompileRelationshipTargets(moverPrim, _tokens->rigExecSpace);
            if (space.size() > 1) {
                return attempt.Fail("RigExecHarmonicsConstraint " +
                                        mover.moverPath.GetString() +
                                        " has more than one rigExec:space",
                                    {mover.moverPath});
            }
            if (!space.empty()) {
                const UsdPrim spacePrim = _stage->GetPrimAtPath(space[0]);
                const TfToken spaceType =
                    spacePrim ? spacePrim.GetTypeName() : TfToken();
                if (spaceType == SchemaTypeName::RigExecControl ||
                    spaceType == SchemaTypeName::RigExecJoint) {
                    constraint.spacePath = space[0];
                }
            }
        } else if (mover.schemaType ==
                   SchemaTypeName::RigExecIkFkMatchConstraint) {
            // It revises no frame a reader sees: its controls' revised
            // frames are presentation, applied where control frames are
            // published. So it has no targets, and reads, in order, the
            // three joints, the IK root, the effector, then its controls --
            // IK control, pole, the three FK controls.
            const auto one = [&](const TfToken &name, SdfPath *out) {
                const SdfPathVector rows =
                    _CompileRelationshipTargets(moverPrim, name);
                if (rows.size() != 1 || !rows[0].IsPrimPath()) {
                    return attempt.Fail("RigExecIkFkMatchConstraint " +
                                            mover.moverPath.GetString() +
                                            " requires exactly one " +
                                            name.GetString(),
                                        {mover.moverPath});
                }
                *out = rows[0];
                return true;
            };
            SdfPath ik, pole, root, effector;
            if (!one(_tokens->rigExecIkControl, &ik) ||
                !one(_tokens->rigExecPoleControl, &pole) ||
                !one(_tokens->rigExecRootControl, &root) ||
                !one(_tokens->rigExecEffector, &effector)) {
                return false;
            }
            const SdfPathVector fk = _CompileRelationshipTargets(
                moverPrim, _tokens->rigExecFkControls);
            const SdfPathVector joints = _CompileRelationshipTargets(
                moverPrim, _tokens->rigExecLimbJoints);
            if (fk.size() != 3 || joints.size() != 3) {
                return attempt.Fail(
                    "RigExecIkFkMatchConstraint " +
                        mover.moverPath.GetString() +
                        " requires three FK controls and three joints",
                    {mover.moverPath});
            }
            const SdfPathVector controls = {ik, pole, fk[0], fk[1], fk[2]};
            for (const SdfPath &target : controls) {
                const UsdPrim control = _stage->GetPrimAtPath(target);
                if (!control ||
                    control.GetTypeName() != SchemaTypeName::RigExecControl) {
                    return attempt.Fail("RigExecIkFkMatchConstraint " +
                                            mover.moverPath.GetString() +
                                            " revises " + target.GetString() +
                                            ", which is not a RigExecControl",
                                        {mover.moverPath});
                }
            }
            if (std::set<SdfPath>(mover.targets.begin(), mover.targets.end()) !=
                std::set<SdfPath>(controls.begin(), controls.end())) {
                return attempt.Fail(
                    "RigExecIkFkMatchConstraint " +
                        mover.moverPath.GetString() +
                        " rigExec:moves must name exactly its IK control, "
                        "pole and FK controls",
                    {mover.moverPath});
            }
            const SdfPathVector reads = {joints[0], joints[1], joints[2], root,
                                         effector,  ik,        pole,      fk[0],
                                         fk[1],     fk[2]};
            for (const SdfPath &read : reads) {
                _FrameSourceBinding binding;
                if (!bindFrameSource(mover.moverPath, read,
                                     "RigExecIkFkMatchConstraint " +
                                         mover.moverPath.GetString() + " input",
                                     &binding)) {
                    return false;
                }
                if (!binding.xformPath.IsEmpty()) {
                    return attempt.Fail(
                        "RigExecIkFkMatchConstraint " +
                            mover.moverPath.GetString() + " reads " +
                            read.GetString() +
                            ", which is not a RigExec transform provider",
                        {mover.moverPath});
                }
                constraint.sources.push_back(binding);
            }
            const SdfPathVector space =
                _CompileRelationshipTargets(moverPrim, _tokens->rigExecSpace);
            if (space.size() > 1) {
                return attempt.Fail("RigExecIkFkMatchConstraint " +
                                        mover.moverPath.GetString() +
                                        " has more than one rigExec:space",
                                    {mover.moverPath});
            }
            if (!space.empty()) {
                const UsdPrim spacePrim = _stage->GetPrimAtPath(space[0]);
                const TfToken spaceType =
                    spacePrim ? spacePrim.GetTypeName() : TfToken();
                if (spaceType == SchemaTypeName::RigExecControl ||
                    spaceType == SchemaTypeName::RigExecJoint) {
                    constraint.spacePath = space[0];
                }
            }
        } else {
            // FBX SingleChainIK names endpoints, not an ordered output list.
            // RigExec joint hierarchy is namespace nesting, so the exact
            // chain is inferred by walking End Joint's ancestors to First
            // Joint. rigExec:moves must declare that complete set.
            const SdfPathVector first = _CompileRelationshipTargets(
                moverPrim, _tokens->rigExecFirstJoint);
            const SdfPathVector end = _CompileRelationshipTargets(
                moverPrim, _tokens->rigExecEndJoint);
            const SdfPathVector effector = _CompileRelationshipTargets(
                moverPrim, _tokens->rigExecEffector);
            if (first.size() != 1 || end.size() != 1 || effector.size() != 1 ||
                !first[0].IsPrimPath() || !end[0].IsPrimPath()) {
                return attempt.Fail(
                    "RigExecSingleChainIkConstraint " +
                        mover.moverPath.GetString() +
                        " requires exactly one firstJoint, endJoint, "
                        "and effector prim",
                    {mover.moverPath});
            }
            SdfPath cursor = end[0];
            while (!cursor.IsEmpty() && cursor != SdfPath::AbsoluteRootPath()) {
                const UsdPrim joint = _stage->GetPrimAtPath(cursor);
                if (!joint ||
                    joint.GetTypeName() != SchemaTypeName::RigExecJoint) {
                    return attempt.Fail(
                        "RigExecSingleChainIkConstraint " +
                            mover.moverPath.GetString() +
                            " endpoint ancestry contains non-joint " +
                            cursor.GetString(),
                        {mover.moverPath});
                }
                constraint.ikChain.push_back(cursor);
                if (cursor == first[0]) {
                    break;
                }
                cursor = cursor.GetParentPath();
            }
            if (constraint.ikChain.empty() ||
                constraint.ikChain.back() != first[0]) {
                return attempt.Fail(
                    "RigExecSingleChainIkConstraint " +
                        mover.moverPath.GetString() + ": endJoint " +
                        end[0].GetString() +
                        " is not a namespace descendant of firstJoint " +
                        first[0].GetString(),
                    {mover.moverPath});
            }
            std::reverse(constraint.ikChain.begin(), constraint.ikChain.end());
            if (constraint.ikChain.size() < 2) {
                return attempt.Fail("RigExecSingleChainIkConstraint " +
                                        mover.moverPath.GetString() +
                                        " needs at least two joints",
                                    {mover.moverPath});
            }
            std::set<SdfPath> declared(mover.targets.begin(),
                                       mover.targets.end());
            std::set<SdfPath> inferred(constraint.ikChain.begin(),
                                       constraint.ikChain.end());
            if (declared != inferred) {
                return attempt.Fail(
                    "RigExecSingleChainIkConstraint " +
                        mover.moverPath.GetString() +
                        " rigExec:moves must equal the complete inferred "
                        "firstJoint-to-endJoint chain",
                    {mover.moverPath});
            }
            constraint.targets = constraint.ikChain;
            if (!bindFrameSource(mover.moverPath, effector[0],
                                 "RigExecSingleChainIkConstraint " +
                                     mover.moverPath.GetString() + " effector",
                                 &constraint.effector)) {
                return false;
            }
            // SingleChain binds no pole input; a malformed dormant pole must
            // not fail compile.
            TfToken solverMode = _tokens->rotatePlane;
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(_tokens->rigExecSolverMode)) {
                a.Get(&solverMode);
            }
            if (solverMode == "rotatePlane") {
                for (const SdfPath &pole : _CompileRelationshipTargets(
                         moverPrim, _tokens->rigExecPoleVectorObjects)) {
                    _FrameSourceBinding binding;
                    if (!bindFrameSource(mover.moverPath, pole,
                                         "RigExecSingleChainIkConstraint " +
                                             mover.moverPath.GetString() +
                                             " pole-vector object",
                                         &binding)) {
                        return false;
                    }
                    constraint.poleObjects.push_back(binding);
                }
            }
        }
        // Precompute the axis masks when provably static for this mover: no
        // property chain revises a mask attribute, none is connected, and each
        // is a single authored opinion. Fallbacks mirror the live read:
        // translation/rotation default on, scale defaults off (FBX).
        {
            const TfToken kMaskNames[3][3] = {
                {_tokens->inputsAffectTranslationX,
                 _tokens->inputsAffectTranslationY,
                 _tokens->inputsAffectTranslationZ},
                {_tokens->inputsAffectRotationX, _tokens->inputsAffectRotationY,
                 _tokens->inputsAffectRotationZ},
                {_tokens->inputsAffectScaleX, _tokens->inputsAffectScaleY,
                 _tokens->inputsAffectScaleZ}};
            const bool kMaskFallback[3] = {true, true, false};
            auto groupStatic = [&](int group) {
                for (int axis = 0; axis < 3; ++axis) {
                    const TfToken &name = kMaskNames[group][axis];
                    // A property path, not a child path: the names carry the
                    // inputs: namespace, which is not path syntax, and the
                    // revised targets are property paths. AppendPath here
                    // builds an invalid path that can never match, silently
                    // freezing every chain-driven mask at its compiled rig value.
                    if (propertyRevisedTargets.count(
                            constraint.moverPath.AppendProperty(name))) {
                        return false;
                    }
                    const UsdAttribute a = moverPrim.GetAttribute(name);
                    if (a) {
                        if (!_AuthoredConnections(a).empty()) {
                            return false; // connected -> driven
                        }
                        std::vector<double> timeSamples;
                        // GetTimeSamples answers success, not a count: the
                        // comparison must be against the samples it filled.
                        if (a.GetTimeSamples(&timeSamples) &&
                            timeSamples.size() > 1) {
                            return false; // animated
                        }
                    }
                    // absent -> schema default -> static
                }
                return true;
            };
            constraint.masksStatic =
                groupStatic(0) && groupStatic(1) && groupStatic(2);
            if (constraint.masksStatic) {
                auto readGroup = [&](int group) {
                    RigExecConstraintAxisMask m;
                    bool *out = &m.x;
                    for (int axis = 0; axis < 3; ++axis, ++out) {
                        bool v = kMaskFallback[group];
                        const UsdAttribute a =
                            moverPrim.GetAttribute(kMaskNames[group][axis]);
                        if (a) {
                            a.Get(&v);
                        }
                        *out = v;
                    }
                    return m;
                };
                constraint.precompTranslation = readGroup(0);
                constraint.precompRotation = readGroup(1);
                constraint.precompScale = readGroup(2);
            }
        }

        for (const SdfPath &target : constraint.targets) {
            newFrameChains[target].push_back(mover.moverPath);
        }
        newFrameConstraints.push_back(std::move(constraint));
    }
    // One order over two kinds of step:
    //   * an aggregate solver that WRITES at least one joint, and
    //   * a pose-domain frame constraint (one that moves a transform provider
    //     rather than points).
    // The order is the reverse of the composed pre-order of the WHOLE RIG --
    // the bottom composed sibling first, a parent after its descendants, the
    // rule _GetMoverExecutionOrder already gives movers -- and NOTHING else
    // breaks a tie. A constraint below a solver therefore runs BEFORE it and
    // feeds it (the incoming frame becomes that solver's rest reference); a
    // constraint above it revises its output, which is what every shipped rig
    // authors and why they are unchanged.
    // Producers absent (see jointWritingSolvers): use find(), never
    // operator[].
    auto &poseStackOrdinal = attempt.poseStackOrdinal;
    {
        std::set<SdfPath> stackSteps = jointWritingSolvers;
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            if (constraint.pointsTarget.IsEmpty()) {
                stackSteps.insert(constraint.moverPath);
            }
        }
        if (!stackSteps.empty()) {
            int ordinal = 0;
            for (const UsdPrim &prim :
                 _GetPoseStackOrder(_stage->GetPrimAtPath(_rigPath))) {
                if (stackSteps.count(prim.GetPath())) {
                    poseStackOrdinal[prim.GetPath()] = ordinal++;
                }
            }
        }
    }
    // A startFrame read is positional: a chain ordered before
    // its provider's writer reads the pre-write frame, and for a rest
    // provider that is the rest pose -- the chain then hangs from nothing
    // and stays frozen with no other diagnostic (for example, finger chains
    // ordered before the arm blend read the rest wrist). Authored and derived
    // starts alike -- the read does not know which.
    for (const UsdPrim &solver : orderedSolvers) {
        if (solver.GetTypeName() != SchemaTypeName::RigExecFkChain &&
            solver.GetTypeName() != SchemaTypeName::RigExecSkeleton2d) {
            continue;
        }
        const UsdRelationship startRel =
            solver.GetRelationship(RigExecStartFrameRelName);
        SdfPathVector starts;
        startRel.GetTargets(&starts);
        attempt.AppendDerivedStart(startRel, &starts);
        if (starts.empty()) {
            continue;
        }
        const int here = attempt.StackOrdinalOf(solver.GetPath());
        if (here < 0) {
            continue; // Guide-only: exec re-derives the order.
        }
        const SdfPath provider = starts[0].GetPrimPath();
        SdfPath culprit;
        const auto owned = newJointBinding.find(provider);
        if (owned != newJointBinding.end()) {
            for (const auto &entry : owned->second) {
                if (entry.first != solver.GetPath() &&
                    attempt.StackOrdinalOf(entry.first) > here) {
                    culprit = entry.first;
                    break;
                }
            }
        }
        if (culprit.IsEmpty()) {
            const auto revised = newFrameChains.find(provider);
            if (revised != newFrameChains.end()) {
                for (const SdfPath &writer : revised->second) {
                    if (writer != solver.GetPath() &&
                        attempt.StackOrdinalOf(writer) > here) {
                        culprit = writer;
                        break;
                    }
                }
            }
        }
        if (culprit.IsEmpty()) {
            continue;
        }
        const std::string message =
            solver.GetPath().GetString() + " reads startFrame " +
            provider.GetString() + " but executes before " +
            culprit.GetString() +
            ", which poses it, so it reads the pre-write frame. Reorder "
            "so the chain executes after the writer -- earlier in the "
            "file, since siblings execute bottom-first (spec §4.2).";
        if (errors) {
            errors->push_back("warning: " + message);
        }
        TF_WARN("%s", message.c_str());
    }
    // A provider carrying pose revisions that is not a joint needs a base
    // frame from somewhere. A Control has computePointFrame like a joint; a
    // plain UsdGeomXformable has no exec computation at all, so its frame
    // comes from its own USD transform and its revised matrix is published
    // back onto the prim for Hydra to inherit.
    auto &newXformDerivedProviders = attempt.newXformDerivedProviders;
    for (const auto &[provider, revisions] : newFrameChains) {
        if (std::find(newJointPaths.begin(), newJointPaths.end(), provider) !=
            newJointPaths.end()) {
            continue; // joints are tapped below
        }
        const UsdPrim providerPrim = _stage->GetPrimAtPath(provider);
        const TfToken type =
            providerPrim ? providerPrim.GetTypeName() : TfToken();
        if (type == SchemaTypeName::RigExecControl ||
            type == SchemaTypeName::RigExecJoint || _IsVolumeWeightType(type)) {
        } else if (providerPrim && UsdGeomXformable(providerPrim)) {
            newXformDerivedProviders.insert(provider);
        } else {
            return attempt.Fail(
                "constraint target " + provider.GetString() +
                    " is neither a RigExec transform provider nor a "
                    "UsdGeomXformable; nothing can carry the revised frame",
                revisions);
        }
    }
    return true;
}

} // namespace rigExec
