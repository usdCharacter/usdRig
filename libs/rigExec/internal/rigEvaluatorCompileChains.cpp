// Compile steps: volume weights, property chains and point revision chains.

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
    (points)
    ((rigExecSpace, SchemaAttrText::rigExecSpace))
    ((rigExecSampleSource, SchemaAttrText::rigExecSampleSource))
    ((rigExecCurve, SchemaAttrText::rigExecCurve))
    ((rigExecInputWeights, SchemaAttrText::rigExecInputWeights))
    ((rigExecBaseWeight, SchemaAttrText::rigExecBaseWeight))
    ((rigExecWeightObject, SchemaAttrText::rigExecWeightObject))
    ((rigExecSources, SchemaAttrText::rigExecSources))
    ((rigExecMode, SchemaAttrText::rigExecMode))
    ((rigExecPointFrame, SchemaAttrText::rigExecPointFrame))
    ((rigExecJointIndices, SchemaAttrText::rigExecJointIndices))
    ((rigExecJointWeights, SchemaAttrText::rigExecJointWeights))
    ((rigExecElementSize, SchemaAttrText::rigExecElementSize))
    (normals)
    ((primvarsNormals, "primvars:normals"))
    ((primvarsNormalsIndices, "primvars:normals:indices"))
    (extent)
    (faceVertexCounts)
    (faceVertexIndices)
    (widths)
    ((rigExecSourceSpace, SchemaAttrText::rigExecSourceSpace))
    ((rigExecShaderPrimvar, SchemaAttrText::rigExecShaderPrimvar))
    ((rigExecShaderDialPrimvar, SchemaAttrText::rigExecShaderDialPrimvar))
    (eyeProjector)
    ((rigExecShaderDialSources, SchemaAttrText::rigExecShaderDialSources))
);

using namespace evaluatorDetail;

namespace {

// Attribute names compile rejects; interned once at load (registry lock).
const TfToken _removedSamplePhase(SchemaAttrText::rigExecSamplePhase);

} // namespace

// Returns true when this weight object, or anything it composes,
// samples the in-flight points.
// The answer has to propagate UP: the graph build loop tests the
// weight object a mover actually binds, which for a composed field is
// the combine, not the sphere inside it. Recording only the leaf left
// exec applying the reference-phase packet while the CPU oracle
// reached a leaf with no in-flight points and failed.
bool
RigExecRigEvaluator::_RegisterVolumeWeights(_CompileAttempt &attempt,
                                            const SdfPath &weightPath)
{
    auto &newFalloffLutOverrides = attempt.newFalloffLutOverrides;
    auto &newCurrentPhaseWeights = attempt.newCurrentPhaseWeights;
    auto &newVolumeWeightProviders = attempt.newVolumeWeightProviders;
    auto &volumeWeightError = attempt.volumeWeightError;
    auto &visiting = attempt.visiting;
    const UsdPrim w = _stage->GetPrimAtPath(weightPath);
    if (!w) {
        return false;
    }
    if (!visiting.insert(weightPath).second)
        return false;
    struct Pop {
        std::set<SdfPath> &s;
        const SdfPath &p;
        ~Pop()
        {
            s.erase(p);
        }
    } pop{visiting, weightPath};

    if (newVolumeWeightProviders.count(weightPath) ||
        newCurrentPhaseWeights.count(weightPath)) {
        // Already walked through another consumer; its answer stands.
        return newCurrentPhaseWeights.count(weightPath) != 0;
    }
    bool isCurrent = false;
    if (_IsVolumeWeightType(w.GetTypeName())) {
        auto requireTargets = [&](const TfToken &rel, size_t exact,
                                  const char *what) {
            SdfPathVector targets;
            if (UsdRelationship r = w.GetRelationship(rel)) {
                r.GetTargets(&targets);
            }
            if (targets.size() > exact) {
                volumeWeightError = weightPath.GetString() + ": " +
                                    rel.GetString() + " must name " + what;
            }
            return targets.size();
        };
        // At most one sampling override; exactly one curve for a
        // curve weight.
        requireTargets(_tokens->rigExecSampleSource, 1,
                       "at most one points source");
        if (w.GetTypeName() == SchemaTypeName::RigExecCurveWeight) {
            SdfPathVector curves;
            if (const UsdRelationship rel =
                    w.GetRelationship(_tokens->rigExecCurve)) {
                rel.GetTargets(&curves);
            }
            if (curves.size() != 1) {
                volumeWeightError =
                    weightPath.GetString() +
                    ": rigExec:curve must name exactly one points source";
            } else {
                const SdfPath pointsPath =
                    curves[0].IsPropertyPath()
                        ? curves[0]
                        : curves[0].AppendProperty(_tokens->points);
                const UsdAttribute points =
                    _stage->GetAttributeAtPath(pointsPath);
                if (!points ||
                    points.GetTypeName() != SdfValueTypeNames->Point3fArray) {
                    volumeWeightError =
                        weightPath.GetString() + ": rigExec:curve target " +
                        curves[0].GetString() +
                        " must resolve to a point3f[] points source";
                }
            }
        }
    }
    if (_IsVolumeWeightType(w.GetTypeName())) {
        // computePointFrame, NOT computeMatrix: the latter is the
        // rest->posed map, so an unanimated volume's is the identity
        // and its field would land at the origin however the prim is
        // placed. See _RigidWorldToLocal in moverKernels.cpp.
        newVolumeWeightProviders.insert(weightPath);

        RigExecValueOverride lutOverride;
        lutOverride.prim = weightPath;
        lutOverride.computation = _computeFalloffLut;
        RigExecFalloffLut lut;
        lut.samples = _BakeFalloffLut(w);
        lutOverride.value = VtValue(lut);
        newFalloffLutOverrides.push_back(std::move(lutOverride));

        // Which points the field measures is the read phase declared
        // on rigExec:weightTarget; the per-mover attribute is refused
        // rather than composing as an inert custom attribute.
        bool inFlight = false;
        std::string phaseError;
        const UsdAttribute old = w.GetAttribute(_removedSamplePhase);
        if (old && old.HasAuthoredValue()) {
            volumeWeightError =
                weightPath.GetString() +
                " authors rigExec:samplePhase, which was replaced by "
                "rigExecReadPhase metadata on rigExec:weightTarget "
                "(reference is base, current is preceding)";
        } else if (!_VolumeWeightSamplesInFlight(w, &inFlight, &phaseError)) {
            volumeWeightError = phaseError;
        } else if (inFlight) {
            newCurrentPhaseWeights.insert(weightPath);
            isCurrent = true;
        }
    }
    for (const TfToken &rel :
         {_tokens->rigExecInputWeights, _tokens->rigExecBaseWeight}) {
        SdfPathVector targets;
        if (UsdRelationship r = w.GetRelationship(rel)) {
            r.GetTargets(&targets);
        }
        for (const SdfPath &t : targets) {
            // Not short-circuited: every reachable weight object
            // still needs its matrix tap and LUT override, so the
            // walk must complete even once the answer is known.
            if (_RegisterVolumeWeights(attempt, t)) {
                isCurrent = true;
            }
        }
    }
    // A composed field is current-phase if anything inside it is, so
    // that the combine a mover actually binds tests true.
    if (isCurrent) {
        newCurrentPhaseWeights.insert(weightPath);
    }
    return isCurrent;
}

void
RigExecRigEvaluator::_CompileVolumeWeights(_CompileAttempt &attempt)
{
    auto &newVolumeWeightPaths = attempt.newVolumeWeightPaths;
    auto &newMovers = attempt.newMovers;
    // Volumetric weight compiled rig state.
    auto &newFalloffLutOverrides = attempt.newFalloffLutOverrides;
    auto &newCurrentPhaseWeights = attempt.newCurrentPhaseWeights;
    auto &newVolumeWeightProviders = attempt.newVolumeWeightProviders;

    // Walks a weight object and everything it composes, gathering what
    // the volumetric types need beyond their computeWeightPacket tap.
    // Depth-limited for the same reason the structure digest is: a cycle
    // is authoring error, and the bound only has to keep this
    // terminating.
    // Structural authoring errors on a volume weight, collected during
    // the walk below and reported before the compiled rig commits.
    // These are cardinality rules on the points-bearing relationships,
    // and they exist because the two evaluation paths CANNOT disagree
    // about them safely: the exec kernel receives a relationship's
    // targets as one flattened value stream, so two targets on
    // rigExec:curve silently concatenate into one polyline with a
    // spurious segment joining them, while the CPU oracle reads targets
    // explicitly and rejects the pair. Catching it here means neither
    // path ever sees the ambiguous authoring.
    auto &volumeWeightError = attempt.volumeWeightError;

    // Descriptor discovery terminates recursive composition here. The common
    // operation graph retains producer edges and diagnoses actual cycles.
    auto &visiting = attempt.visiting;

    // A placed volume is a guide output even before it is bound to a mover.
    // Register every discovered volume first; the recursive consumer walks
    // below naturally deduplicate against this map. This also applies the same
    // CurveWeight cardinality validation to standalone and consumed volumes.
    for (const SdfPath &volumePath : newVolumeWeightPaths) {
        _RegisterVolumeWeights(attempt, volumePath);
    }

    // Gather volumetric compiled rig state for every common mover envelope, not only
    // point-graph revisions. Geometry-domain constraints publish directly
    // after the pose walk and therefore never appear in _graphChains, but a
    // sphere/plane/curve field bound to one still needs the same placement tap
    // and falloff override as a geometry mover.
    for (const RigExecMoverRecord &mover : newMovers) {
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        SdfPathVector objects;
        if (const UsdRelationship rel =
                moverPrim.GetRelationship(_tokens->rigExecWeightObject)) {
            rel.GetTargets(&objects);
        }
        if (objects.size() == 1) {
            _RegisterVolumeWeights(attempt, objects[0]);
        }
    }
}

bool
RigExecRigEvaluator::_CompilePropertyChains(_CompileAttempt &attempt)
{
    _CompileFailure *failure = attempt.failure;
    auto &newMovers = attempt.newMovers;
    auto &inertMovers = attempt.inertMovers;
    const std::vector<UsdPrim> &orderedSolvers = attempt.aggregateSolvers;
    // Property-domain chains, from the same mover execution walk.
    // Nothing to bind and nothing to tap: a math mover's inputs are all
    // authored on itself, and the chain's base is the target attribute's own
    // authored value. That is exactly what makes the chain evaluable BEFORE
    // exec runs, and therefore what lets its result be supplied to exec as a
    // value override -- which is how a clamped weight actually reaches the
    // solver that reads it instead of being reimplemented inside that
    // solver's kernel.
    auto &newPropertyChains = attempt.newPropertyChains;
    auto &newPropertyChainOrder = attempt.newPropertyChainOrder;
    auto &newPhasedConnections = attempt.newPhasedConnections;
    if (!_CompilePropertyChains(newMovers, newPropertyChains,
                                newPropertyChainOrder, newPhasedConnections,
                                orderedSolvers, inertMovers, failure)) {
        return attempt.Fail(failure->message, failure->operations);
    }
    return true;
}

bool
RigExecRigEvaluator::_CompilePointGraph(_CompileAttempt &attempt)
{
    auto &newMovers = attempt.newMovers;
    auto &newPropertyChains = attempt.newPropertyChains;
    // Bind the persistent point graph from the composed execution walk.
    // Geometry constraints contribute solved matrix packets to this same
    // chain. Compilation resolves bindings without authoring scene data.
    auto &newGraphChains = attempt.newGraphChains;
    for (const RigExecMoverRecord &mover : newMovers) {
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        TfToken curveMode;
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_tokens->rigExecMode)) {
            a.Get(&curveMode);
        }
        std::optional<RigExecRevisionOp> op =
            RigExecRevisionOpForSchema(mover.schemaType, curveMode);
        if (!op && _IsSourceFrameConstraintType(mover.schemaType)) {
            op = RigExecRevisionOp::Matrix;
        }
        if (!op) {
            // A mover row that resolves no op for a points write would leave
            // the chain unrevised.
            if (RigExecFindMoverHandler(mover.schemaType)) {
                for (const SdfPath &target : mover.targets) {
                    if (target.IsPropertyPath() &&
                        target.GetNameToken() == "points") {
                        return attempt.Fail(
                            mover.schemaType.GetString() + " " +
                                mover.moverPath.GetString() +
                                " resolves no point op for mode '" +
                                curveMode.GetString() + "'",
                            {mover.moverPath});
                    }
                }
            }
            continue;
        }
        for (const SdfPath &target : mover.targets) {
            if (!target.IsPropertyPath() || target.GetNameToken() != "points") {
                continue;
            }
            _GraphRevision revision;
            revision.moverPath = mover.moverPath;
            revision.target = target;
            revision.op = *op;
            revision.binding =
                RigExecResolveRevisionBinding(moverPrim, target, {});
            // The phase each mover's own handler resolved from the input
            // that names its providers: rigExec:transform, rigExec:influences
            // or rigExec:driverTransforms.
            revision.transformFinalPhase =
                revision.binding.transformPhase.kind ==
                RigExecReadPhaseKind::Final;
            {
                TfToken pointFrame;
                if (const UsdAttribute a =
                        moverPrim.GetAttribute(_tokens->rigExecPointFrame)) {
                    a.Get(&pointFrame);
                }
                revision.transformPosedPoints = pointFrame == "posed";
            }
            if (!revision.binding.weightObject.IsEmpty()) {
                _RegisterVolumeWeights(attempt, revision.binding.weightObject);
            }
            // Declared leaves capture blend samples. A computeBlendChannel
            // request here would duplicate that sampling.
            if (revision.op == RigExecRevisionOp::Skin) {
                // Whether the per-point layout can change WITHIN this compiled rig
                // is a question about the stage, so it is answered here once
                // rather than guessed at per frame. The three ways it can:
                // an authored time sample (the arrays differ per time code),
                // an authored connection (the value comes from somewhere
                // else, which may itself be animated), and a property chain
                // writing the attribute (the evaluator computes it per
                // generation). Anything else is compiled rig constant by the same
                // definition the digest uses, so the layout is resolved once
                // and shared instead of re-read, re-copied and re-validated
                // on every frame.
                revision.skinTopologyFixed = true;
                for (const TfToken &name : {_tokens->rigExecJointIndices,
                                            _tokens->rigExecJointWeights,
                                            _tokens->rigExecElementSize}) {
                    const SdfPath propertyPath =
                        mover.moverPath.AppendProperty(name);
                    if (newPropertyChains.count(propertyPath)) {
                        revision.skinTopologyFixed = false;
                        break;
                    }
                    const UsdAttribute a = moverPrim.GetAttribute(name);
                    if (a && (a.ValueMightBeTimeVarying() ||
                              a.HasAuthoredConnections())) {
                        revision.skinTopologyFixed = false;
                        break;
                    }
                }
            }
            newGraphChains[target].push_back(revision);
        }
    }
    return true;
}

void
RigExecRigEvaluator::_CompileSampleReads(_CompileAttempt &attempt)
{
    auto &newMovers = attempt.newMovers;
    auto &newGraphChains = attempt.newGraphChains;
    // Resolve sample-relative preceding reads against the global mover walk.
    // Samples can belong to a different chain, or multiple blend applications;
    // each application gets its own checkpoint, independent of sample identity.
    std::map<SdfPath, int> applicationOrdinals;
    for (const auto &mover : newMovers)
        applicationOrdinals[mover.moverPath] = mover.ordinal;
    for (auto &[target, revisions] : newGraphChains) {
        for (auto &revision : revisions) {
            for (auto &[input, samples] : revision.binding.blendSamples) {
                for (auto &sample : samples) {
                    const auto producers = newGraphChains.find(sample.points);
                    if (producers == newGraphChains.end()) {
                        sample.phase = RigExecReadPhase();
                    } else if (sample.phase.kind ==
                               RigExecReadPhaseKind::Preceding) {
                        SdfPath checkpoint;
                        for (const auto &producer : producers->second) {
                            if (applicationOrdinals[producer.moverPath] <
                                applicationOrdinals[revision.moverPath])
                                checkpoint = producer.moverPath;
                        }
                        sample.phase =
                            checkpoint.IsEmpty()
                                ? RigExecReadPhase()
                                : RigExecReadPhase{RigExecReadPhaseKind::AtPrim,
                                                   checkpoint};
                    }
                }
            }
        }
    }
}

bool
RigExecRigEvaluator::_CompileDerivedMaintenance(_CompileAttempt &attempt)
{
    auto &newSurfaceProjectors = attempt.newSurfaceProjectors;
    auto &newGraphChains = attempt.newGraphChains;
    // Derived maintenance, mirroring SolverSchedule.PointGraph: for every
    // moved points target whose gprim authors normals or extent, synthesize
    // the recompute revision. There is no authored mover, so the gprim itself
    // stands in as the parameter source -- it supplies the stage for the
    // static topology reads and has no inputs:enabled, so the revision is
    // enabled. Vertex-normal recomputation is mesh-only; the compiler already
    // rejects authored normals on a non-mesh points target.
    auto &newGraphDerivedChains = attempt.newGraphDerivedChains;
    for (const auto &[pointsTarget, revisions] : newGraphChains) {
        const SdfPath ownerPath = pointsTarget.GetPrimPath();
        const UsdPrim owner = _stage->GetPrimAtPath(ownerPath);
        if (!owner) {
            continue;
        }
        for (const bool isNormals : {true, false}) {
            SdfPath derivedTarget = ownerPath.AppendProperty(
                isNormals ? _tokens->normals : _tokens->extent);
            // On a mesh, primvars:normals, when authored, wins over normals.
            if (isNormals && UsdGeomMesh(owner)) {
                const SdfPath primvarTarget =
                    ownerPath.AppendProperty(_tokens->primvarsNormals);
                if (const UsdAttribute primvar =
                        _stage->GetAttributeAtPath(primvarTarget)) {
                    if (primvar.HasAuthoredValue()) {
                        derivedTarget = primvarTarget;
                    }
                }
            }
            const UsdAttribute authored =
                _stage->GetAttributeAtPath(derivedTarget);
            if (!authored || !authored.HasAuthoredValue()) {
                continue;
            }
            // Vertex-normal recomputation is mesh-only. Silently leaving
            // authored normals stale on a moved Points/BasisCurves target
            // would break the automatic-maintenance contract, so reject the
            // configuration rather than skip it.
            if (isNormals && !UsdGeomMesh(owner)) {
                return attempt.Fail(
                    "authored normals on non-mesh points target " +
                    ownerPath.GetString() +
                    " cannot be maintained (vertex-normal recomputation is "
                    "mesh-only, spec §7.6 revised); remove the authored "
                    "normals");
            }
            _GraphRevision derived;
            derived.moverPath = ownerPath;
            derived.target = derivedTarget;
            derived.op = isNormals ? RigExecRevisionOp::RecomputeNormals
                                   : RigExecRevisionOp::RecomputeExtent;
            derived.binding.moverPath = ownerPath;
            derived.binding.target = derivedTarget;
            derived.binding.topologyCounts =
                ownerPath.AppendProperty(_tokens->faceVertexCounts);
            derived.binding.topologyIndices =
                ownerPath.AppendProperty(_tokens->faceVertexIndices);
            if (isNormals) {
                const UsdAttribute normalsIndices = _stage->GetAttributeAtPath(
                    ownerPath.AppendProperty(_tokens->primvarsNormalsIndices));
                TfToken interpolation, orientation;
                authored.GetMetadata(UsdGeomTokens->interpolation,
                                     &interpolation);
                const UsdGeomMesh mesh(owner);
                mesh.GetOrientationAttr().Get(&orientation);
                std::string why;
                if (!RigExecResolveNormalsLayout(
                        derivedTarget, interpolation,
                        derivedTarget.GetNameToken() ==
                                _tokens->primvarsNormals &&
                            normalsIndices && normalsIndices.HasAuthoredValue(),
                        orientation, &derived.binding, &why)) {
                    return attempt.Fail(why);
                }
                // A walk only for a topology no frame can change: unsampled
                // and unconnected. An edit of it recompiles (the digest
                // hashes it), so the kernel trusts the walk.
                const UsdAttribute counts = mesh.GetFaceVertexCountsAttr(),
                                   indices = mesh.GetFaceVertexIndicesAttr();
                if (!counts.ValueMightBeTimeVarying() &&
                    !indices.ValueMightBeTimeVarying() &&
                    !counts.HasAuthoredConnections() &&
                    !indices.HasAuthoredConnections()) {
                    VtVec3fArray restPoints;
                    VtIntArray countValues, indexValues;
                    mesh.GetPointsAttr().Get(&restPoints,
                                             UsdTimeCode::EarliestTime());
                    counts.Get(&countValues);
                    indices.Get(&indexValues);
                    RigExecAttachNormalsTopology(&derived.binding,
                                                 restPoints.size(), countValues,
                                                 indexValues);
                }
            } else {
                // The authoritative winning widths, when authored: they
                // widen the extent bound (resolvedWidths, moverKernels.cpp).
                const SdfPath widthsPath =
                    ownerPath.AppendProperty(_tokens->widths);
                if (const UsdAttribute w =
                        _stage->GetAttributeAtPath(widthsPath)) {
                    if (w.HasAuthoredValue()) {
                        derived.binding.widths = widthsPath;
                    }
                }
            }
            newGraphDerivedChains[pointsTarget].push_back(derived);
        }
    }
    // Surface projectors (RigExecSurfaceProjector): derived MATRIX targets
    // on the chain they ride, measured from that chain's final points the
    // way normals and extent are. The shader matrix and the packed dials
    // are each one target, published as a primvar of the surface and
    // never authored back.
    for (const _SurfaceProjectorRecord &projector : newSurfaceProjectors) {
        const UsdPrim prim = _stage->GetPrimAtPath(projector.path);
        const SdfPath meshPath = projector.target.GetPrimPath();
        const UsdPrim mesh = _stage->GetPrimAtPath(meshPath);
        if (!prim || !mesh || !UsdGeomMesh(mesh)) {
            return attempt.Fail(
                projector.path.GetString() +
                    ": a surface projector must move one mesh's "
                    "points",
                {projector.path});
        }
        if (!newGraphChains.count(projector.target)) {
            TF_WARN("%s projects onto %s, which no mover deforms; it "
                    "publishes nothing",
                    projector.path.GetText(), meshPath.GetText());
            continue;
        }
        // The mesh's transform relative to the asset places the ray when the
        // source names no sibling space. It is a stage transform the rig does
        // not own, so it is captured here and must not vary over time; the
        // asset root and its ancestors cancel out of it.
        const SdfPath assetRootPath = _rigPath.GetParentPath();
        for (UsdPrim p = mesh;
             p && !p.IsPseudoRoot() && p.GetPath() != assetRootPath;
             p = p.GetParent()) {
            if (const UsdGeomXformable xformable{p}) {
                if (xformable.TransformMightBeTimeVarying()) {
                    return attempt.Fail(
                        projector.path.GetString() + " projects onto " +
                            meshPath.GetString() +
                            ", whose transform is animated; a surface "
                            "projector needs a static surface "
                            "transform",
                        {projector.path});
                }
            }
        }
        UsdGeomXformCache meshXforms(UsdTimeCode::Default());
        const UsdPrim assetRoot = _stage->GetPrimAtPath(assetRootPath);
        const GfMatrix4d assetWorld =
            assetRoot && !assetRoot.IsPseudoRoot()
                ? meshXforms.GetLocalToWorldTransform(assetRoot)
                : GfMatrix4d(1.0);
        const GfMatrix4d assetToMesh =
            assetWorld * meshXforms.GetLocalToWorldTransform(mesh).GetInverse();
        // Source, its sibling space and the rig's space, each at most one
        // frame provider: bound as the transform, transformSpace and carry
        // providers, which every evaluator already reads per phase.
        const auto oneProvider = [&](const TfToken &name, SdfPath *out) {
            const SdfPathVector targets =
                _CompileRelationshipTargets(prim, name);
            if (targets.size() > 1) {
                return attempt.Fail(projector.path.GetString() +
                                        " has more than "
                                        "one " +
                                        name.GetString(),
                                    {projector.path});
            }
            if (!targets.empty()) {
                if (!_IsFrameProvider(
                        _stage->GetPrimAtPath(targets[0].GetPrimPath()))) {
                    return attempt.Fail(projector.path.GetString() + " " +
                                            name.GetString() + " target " +
                                            targets[0].GetString() +
                                            " is not a RigExec frame provider",
                                        {projector.path});
                }
                *out = targets[0].GetPrimPath();
            }
            return true;
        };
        _GraphRevision base;
        base.moverPath = projector.path;
        base.binding.moverPath = projector.path;
        base.binding.base = projector.target;
        base.binding.topologyCounts =
            meshPath.AppendProperty(_tokens->faceVertexCounts);
        base.binding.topologyIndices =
            meshPath.AppendProperty(_tokens->faceVertexIndices);
        base.binding.meshWorldInverse = assetToMesh;
        if (!oneProvider(_tokens->rigExecSources, &base.binding.transform) ||
            !oneProvider(_tokens->rigExecSourceSpace,
                         &base.binding.transformSpace) ||
            !oneProvider(_tokens->rigExecSpace, &base.binding.carrySpace)) {
            return false;
        }
        const auto primvarTarget = [&](const TfToken &attr,
                                       const TfToken &fallback) {
            TfToken name = fallback;
            if (const UsdAttribute a = prim.GetAttribute(attr)) {
                a.Get(&name);
            }
            return name.IsEmpty() ? SdfPath()
                                  : meshPath.AppendProperty(TfToken(
                                        "primvars:" + name.GetString()));
        };
        const SdfPath shaderTarget =
            primvarTarget(_tokens->rigExecShaderPrimvar, _tokens->eyeProjector);
        if (!shaderTarget.IsEmpty()) {
            _GraphRevision derived = base;
            derived.target = shaderTarget;
            derived.binding.target = shaderTarget;
            derived.op = RigExecRevisionOp::SurfaceProjector;
            newGraphDerivedChains[projector.target].push_back(derived);
        }
        const SdfPath dialTarget =
            primvarTarget(_tokens->rigExecShaderDialPrimvar, TfToken());
        if (!dialTarget.IsEmpty()) {
            _GraphRevision derived = base;
            derived.target = dialTarget;
            derived.binding.target = dialTarget;
            // Packs dial values only: no frames and no surface, so no mesh
            // topology for the frame cache to sample with every frame.
            derived.binding.transform = SdfPath();
            derived.binding.transformSpace = SdfPath();
            derived.binding.carrySpace = SdfPath();
            derived.binding.topologyCounts = SdfPath();
            derived.binding.topologyIndices = SdfPath();
            derived.op = RigExecRevisionOp::ShaderDials;
            for (const SdfPath &dial : _CompileRelationshipTargets(
                     prim, _tokens->rigExecShaderDialSources)) {
                if (!dial.IsPropertyPath()) {
                    return attempt.Fail(
                        projector.path.GetString() +
                            " rigExec:shaderDialSources names " +
                            dial.GetString() + ", not a property",
                        {projector.path});
                }
                if (derived.binding.shaderDials.size() == 16) {
                    TF_WARN("%s names more than sixteen shader dials; the "
                            "rest do not fit the primvar",
                            projector.path.GetText());
                    break;
                }
                derived.binding.shaderDials.push_back(dial);
            }
            newGraphDerivedChains[projector.target].push_back(derived);
        }
    }

    // One output cannot be both a spatial frame and packed scalar dials.
    // Same-semantic overwrites retain their existing ordered behavior.
    std::map<SdfPath, std::pair<RigExecRevisionOp, SdfPath>> matrixOutputs;
    for (const auto &[pointsTarget, revisions] : newGraphDerivedChains) {
        for (const _GraphRevision &revision : revisions) {
            if (revision.op != RigExecRevisionOp::SurfaceProjector &&
                revision.op != RigExecRevisionOp::ShaderDials) {
                continue;
            }
            const auto [found, inserted] = matrixOutputs.emplace(
                revision.target,
                std::make_pair(revision.op, revision.moverPath));
            if (!inserted && found->second.first != revision.op) {
                const SdfPath spatial =
                    revision.op == RigExecRevisionOp::SurfaceProjector
                        ? revision.moverPath
                        : found->second.second;
                const SdfPath dials =
                    revision.op == RigExecRevisionOp::ShaderDials
                        ? revision.moverPath
                        : found->second.second;
                return attempt.Fail(
                    "shader matrix output " + revision.target.GetString() +
                        " is a spatial SurfaceProjector frame from " +
                        spatial.GetString() + " and packed ShaderDials from " +
                        dials.GetString() +
                        "; choose different "
                        "rigExec:shaderPrimvar and "
                        "rigExec:shaderDialPrimvar names",
                    {spatial, dials});
            }
        }
    }
    return true;
}

} // namespace rigExec
