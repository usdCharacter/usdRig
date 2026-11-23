// Whether a compiled rig can be baked, and why not.
#include "rigExec/internal/weightField.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/inputReplay.h"

#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/bakedProgramShared.h"
#include "rigExec/internal/bakedBuildAttempt.h"
#include "rigExec/internal/projectorCaptureNotice.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/oracleDispatch.h"
#include "rigExec/internal/bakedSchedule.h"
#include "rigExec/internal/bakedOpGraph.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/frozenContextInternal.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/movers/pathOperation.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/api/types.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecSchema/schemaTokens.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

using namespace bakedProgramDetail;

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((rigExecBaseWeight, SchemaAttrText::rigExecBaseWeight))
    ((rigExecInputWeights, SchemaAttrText::rigExecInputWeights))
);

// The constraint operators the program expresses. A type outside this set
// makes the compiled rig unbakeable rather than silently passing through.
bool
_IsBakedConstraintType(const TfToken &type)
{
    const bool baked = type == SchemaType::RigExecPositionConstraint ||
                       type == SchemaType::RigExecRotationConstraint ||
                       type == SchemaType::RigExecScaleConstraint ||
                       type == SchemaType::RigExecParentConstraint ||
                       type == SchemaType::RigExecAimConstraint ||
                       type == SchemaType::RigExecSingleChainIkConstraint ||
                       type == SchemaType::RigExecIkFkMatchConstraint ||
                       type == SchemaType::RigExecHarmonicsConstraint;
    // RigExecRigEvaluator::GetConstraintOperatorTypeNames() is the authority
    // on which operators exist; this is the subset the program can express,
    // and it must stay a subset. A name here the evaluator does not register
    // is a typo that refuses nothing and bakes nothing, and it would read as
    // an operator that simply never takes the baked path.
    TF_VERIFY(!baked || RigExecRigEvaluator::IsConstraintOperatorType(type),
              "rigExec: %s is baked but is not a registered constraint "
              "operator",
              type.GetText());
    return baked;
}

bool
_IsBakedSolverType(const TfToken &type)
{
    return type == SchemaType::RigExecFkChain ||
           type == SchemaType::RigExecTwoBoneIk ||
           type == SchemaType::RigExecBlendPointFrames ||
           type == SchemaType::RigExecSplineIk ||
           type == SchemaType::RigExecTwistDistribution ||
           type == SchemaType::RigExecRibbon ||
           type == SchemaType::RigExecSkeleton2d;
}

// The weight-object schemas the program can build a packet for.
// It is about the BUILDER and nothing else. The volumetric three are here
// because RigExecBuildVolumeWeightPacket is one of the builders; volumes
// bake, and the program places them itself (VolumePlacements). The only
// tap-related check is the one-way guard above (_WalkVolumesWithoutASlot):
// every volume the walk places needs a provider slot.
bool
_IsBakedWeightType(const TfToken &type)
{
    return type == SchemaTypeName::RigExecStaticWeight ||
           type == SchemaTypeName::RigExecDynamicWeight ||
           type == SchemaTypeName::RigExecCombineWeight ||
           _IsVolumeWeightTypeName(type);
}

} // namespace

// Bakeability.

bool
RigExecRevisionOpBakes(RigExecRevisionOp op, bool derived)
{
    if (derived) {
        return op == RigExecRevisionOp::RecomputeExtent ||
               op == RigExecRevisionOp::RecomputeNormals ||
               RigExecIsDerivedMatrixOp(op);
    }
    // Every operation whose whole packet the per-frame assembler reads off
    // the stage: the program hands RigExecAssembleParameters the same binding
    // and resolved inputs the dynamic walk does, and RigExecRunRevisionKernel
    // is the same kernel.
    switch (op) {
    case RigExecRevisionOp::Skin:
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::BlendShape:
    case RigExecRevisionOp::External:
    case RigExecRevisionOp::EmitGuidePoints:
    case RigExecRevisionOp::Ribbon:
    case RigExecRevisionOp::Wire:
    case RigExecRevisionOp::VolumeCorrect:
    case RigExecRevisionOp::Smooth:
    case RigExecRevisionOp::DeltaMush:
    case RigExecRevisionOp::Wrinkle:
    case RigExecRevisionOp::Lattice:
    case RigExecRevisionOp::SurfaceProject:
        return true;
    default:
        return false;
    }
}

bool
RigExecBakedProgram::IsBakeable(const RigExecRigEvaluator &evaluator,
                                std::vector<std::string> *reasons)
{
    // The refusal list IS the answer here -- the walk below is the same
    // whether or not anybody wants to read the sentences -- so a caller
    // that passes nothing is given a scratch vector and its refusals are
    // dropped on return; see _WantsBakeRefusalReasons for who keeps them.
    std::vector<std::string> local;
    std::vector<std::string> &out = reasons ? *reasons : local;
    const size_t before = out.size();
    const RigExecRigEvaluator &E = evaluator;
    std::map<SdfPath, std::set<SdfPath>> nativeCheckpoints;
    for (const auto &[target, writers] : E._frameChains)
        nativeCheckpoints[target].insert(writers.begin(), writers.end());
    auto say = [&out](const std::string &what, const SdfPath &where) {
        out.push_back(what + ": " + where.GetString());
    };

    if (!E._rigState.Compiled()) {
        out.push_back("the rig is not compiled");
        return false;
    }

    // The step reads the driver's frame, and its parent's, out of the slots,
    // so both have to BE slots. Compile already refused a driver that is not a
    // joint or control of the rig; this is the same fact stated against the
    // program's own table.
    for (const RigExecRigEvaluator::_PoseInterpolator &record :
         E._poseInterpolators) {
        // A NUMERIC driver reads dials rather than a frame: it names no
        // driver prim, so there is no slot for one to be. An ML mover reads
        // both.
        if (!record.driverAttributes.empty() &&
            !(record.descriptor && record.descriptor->ml &&
              !record.driver.IsEmpty())) {
            continue;
        }
        if (!E._nativeProviderPaths.count(record.driver)) {
            say("pose interpolator driver is not a pose provider",
                record.driver);
        }
        if (!record.driverParent.IsEmpty() &&
            !E._nativeProviderPaths.count(record.driverParent)) {
            say("pose interpolator driver parent is not a pose provider",
                record.driverParent);
        }
        for (size_t k = 0; k < record.extraDrivers.size(); ++k) {
            if (!E._nativeProviderPaths.count(record.extraDrivers[k])) {
                say("pose interpolator driver is not a pose provider",
                    record.extraDrivers[k]);
            }
            if (!record.extraDriverParents[k].IsEmpty() &&
                !E._nativeProviderPaths.count(record.extraDriverParents[k])) {
                say("pose interpolator driver parent is not a pose provider",
                    record.extraDriverParents[k]);
            }
        }
    }

    const UsdTimeCode probe = _ProbeTime(E._stage);
    // A property chain RECOMPUTES its target every generation, so a value
    // read once at bake time is not that target's value -- it is whatever
    // the previous generation happened to leave behind. Inputs the program
    // re-reads per frame resolve through the chain and are fine; the ones
    // resolved once into the rest chain and the default-space ladder are
    // not, so a chain aimed at one of those refuses the bake.
    std::set<SdfPath> chainTargets;
    for (const auto &[target, revisions] : E._propertyChains) {
        chainTargets.insert(target);
    }
    // A space switch replaces a provider's parent with one the ladder does
    // not know about. The compose reads every frame it needs at a version
    // bound at Build (RigExecBakedProgramImpl::SpaceSwitch::FrameVersion)
    // so all it needs here is a slot for the target and for each source.
    for (const RigExecRigEvaluator::_SpaceSwitch &sw : E._spaceSwitches) {
        if (!E._nativeProviderPaths.count(sw.target)) {
            say("space switch on a provider with no slot", sw.target);
            continue;
        }
        for (const auto &source : sw.sources) {
            if (source.path.IsEmpty()) {
                continue; // world: identity, no slot needed
            }
            if (!E._nativeProviderPaths.count(source.path)) {
                say("space switch source has no slot", source.path);
            }
        }
    }
    for (const SdfPath &path : E._nativeProviderPaths) {
        const UsdPrim prim = E._stage->GetPrimAtPath(path);
        if (!prim) {
            say("pose provider has no prim", path);
            continue;
        }
        const TfToken type = prim.GetTypeName();
        // The volumetric three are RigExecXformables, so the pose walk
        // composes them into a slot exactly as it composes a joint -- with
        // one difference the compose has to know about, which is that their
        // scale avars are read and discarded (RigExecBakedProgramImpl::
        // noScaleAvars).
        if (type != SchemaTypeName::RigExecJoint &&
            type != SchemaTypeName::RigExecControl &&
            !_IsVolumeWeightTypeName(type)) {
            say("provider type not baked (" + type.GetString() + ")", path);
        }
        (void)probe;
    }
    for (const SdfPath &joint : E._jointPaths) {
        if (!E._nativeProviderPaths.count(joint)) {
            say("joint is not a seeded pose provider", joint);
        }
    }
    for (const SdfPath &control : E._controlPaths) {
        if (!E._nativeProviderPaths.count(control)) {
            say("control is not a seeded pose provider", control);
        }
    }

    std::set<SdfPath> batched;
    for (const auto &batch : E._solverBatches) {
        for (const SdfPath &solverPath : batch.solvers) {
            batched.insert(solverPath);
            const UsdPrim prim = E._stage->GetPrimAtPath(solverPath);
            const TfToken type = prim ? prim.GetTypeName() : TfToken();
            if (!_IsBakedSolverType(type)) {
                say("solver type not baked (" + type.GetString() + ")",
                    solverPath);
            }
            // A TwoBoneIk that does not bind three joint rests is not refused
            // here: it is not a rig the program cannot express. The
            // computation warns and publishes an empty aggregate, and
            // bakeSolver reproduces that through Solver::degenerate. One
            // expression of the rule, in the place that needs its answer.
        }
    }
    // An aggregate solver no batch runs is not a rig the program cannot
    // express: the dynamic path computes it inside the guide request,
    // against the walk's FINAL frames, and the program runs it as a Solve
    // step after the walk for the same reason. Its TYPE still has to be one
    // the program expresses, which the batch loop above never asked about
    // because it never saw it.
    for (const SdfPath &solverPath : E._nativeGuideSolvers) {
        if (batched.count(solverPath)) {
            continue;
        }
        const UsdPrim prim = E._stage->GetPrimAtPath(solverPath);
        const TfToken type = prim ? prim.GetTypeName() : TfToken();
        if (!_IsBakedSolverType(type)) {
            say("solver type not baked (" + type.GetString() + ")", solverPath);
        }
    }

    // A weight object is a COMPOSITION -- a dynamic weight remaps a base, a
    // combine folds a list -- so every question about one is a question about
    // its closure and not about the object a mover or a constraint happens to
    // name. Both walks below carry a visited set of their own rather than
    // trusting the compile pass to have rejected a cycle first: a
    // bakeability check that only terminates because somebody else checked is
    // not one to leave in place.
    const auto walkWeights =
        [&E](const SdfPath &root,
             const std::function<bool(const UsdPrim &)> &visit) {
            std::set<SdfPath> seen;
            std::function<void(const SdfPath &)> walk =
                [&](const SdfPath &path) {
                    if (path.IsEmpty() || !seen.insert(path).second) {
                        return;
                    }
                    const UsdPrim prim = E._stage->GetPrimAtPath(path);
                    if (!prim || !visit(prim)) {
                        return;
                    }
                    for (const TfToken &name : {_tokens->rigExecBaseWeight,
                                                _tokens->rigExecInputWeights}) {
                        for (const SdfPath &input : _Targets(prim, name)) {
                            walk(input);
                        }
                    }
                };
            walk(root);
        };
    const auto sayUnbakedWeights = [&](const SdfPath &root) {
        walkWeights(root, [&](const UsdPrim &prim) {
            const TfToken type = prim.GetTypeName();
            if (!_IsBakedWeightType(type)) {
                say("weight object type not baked (" + type.GetString() + ")",
                    prim.GetPath());
                return false;
            }
            return true;
        });
    };

    for (const auto &constraint : E._frameConstraints) {
        // A transform-domain matrix mover rides the constraint walk without
        // being a constraint operator, so it is admitted by name here
        // rather than through the operator registry's subset check.
        if (constraint.schemaType != SchemaTypeName::RigExecMatrixMover &&
            !_IsBakedConstraintType(constraint.schemaType)) {
            say("constraint type not baked (" +
                    constraint.schemaType.GetString() + ")",
                constraint.moverPath);
            continue;
        }
        // Envelope fields use singleton samples and the consuming relation's
        // selected Base/Final placement. The common graph orders placement
        // and field producers together with the constraint.
        sayUnbakedWeights(constraint.weightObject);
        // An IK/FK match revises no frame a reader sees, so it has none.
        if (constraint.targets.empty() &&
            constraint.schemaType !=
                SchemaTypeName::RigExecIkFkMatchConstraint) {
            say("constraint names no target", constraint.moverPath);
            continue;
        }
        // Either provider family: an exec-seeded one, or a plain Xformable
        // the program seeds from the stage in its prologue. Both take a slot
        // in the same table, which is what the dynamic walk's one frame map
        // holds. EVERY target, not only the first: a SingleChainIK revises
        // its whole chain atomically, and the dynamic walk records every
        // target of every constraint whatever it revises.
        for (const SdfPath &target : constraint.targets) {
            if (!E._nativeProviderPaths.count(target) &&
                !E._xformDerivedProviders.count(target)) {
                say("constraint target is not a seeded pose provider", target);
            }
        }
    }

    // Every volume the walk places needs a program slot; Build checks the
    // same set from its own table.
    {
        std::set<SdfPath> programVolumes;
        const auto noteVolume = [&](const SdfPath &path) {
            const UsdPrim prim = E._stage->GetPrimAtPath(path);
            if (prim && _IsVolumeWeightTypeName(prim.GetTypeName())) {
                programVolumes.insert(path);
            }
        };
        for (const SdfPath &path : E._nativeProviderPaths) {
            noteVolume(path);
        }
        for (const SdfPath &path : E._xformDerivedProviders) {
            noteVolume(path);
        }
        for (const SdfPath &path : _WalkVolumesWithoutASlot(
                 programVolumes, E._nativeVolumeProviders)) {
            say("a volume weight the walk places has no program slot", path);
        }
    }

    // A read phase that names a SOLVER checkpoint reads a frame record on the
    // solver's commit, which exists only for a joint the solver's batch
    // commits. The compile names a writer of the joint, so this is a
    // consistency guard; Build checks the same condition.
    for (const SdfPath &solver : _SolverCheckpointsWithoutAnOutput(
             nativeCheckpoints, E._solverDependencies, E._solverJoints,
             _BatchedSolvers(E._solverBatches))) {
        say("read phase names a solver checkpoint the walk does not commit "
            "for that joint",
            solver);
    }

    auto checkRevision = [&](const RigExecRigEvaluator::_GraphRevision &r,
                             bool derived) {
        if (!RigExecRevisionOpBakes(r.op, derived)) {
            say(std::string("mover operation not baked (") +
                    RigExecBakedOpName(r.op) + ")",
                r.moverPath);
            return;
        }
        sayUnbakedWeights(r.binding.weightObject);
        if (r.op == RigExecRevisionOp::Skin) {
            const UsdPrim prim = E._stage->GetPrimAtPath(r.moverPath);
            for (const SdfPath &influence : r.binding.influences) {
                if (!E._nativeProviderPaths.count(influence)) {
                    say("skin influence is not a seeded pose provider",
                        influence);
                }
            }
        }
    };
    for (const auto &[target, revisions] : E._graphChains) {
        for (const auto &revision : revisions) {
            checkRevision(revision, /* derived = */ false);
        }
    }
    for (const auto &[target, revisions] : E._graphDerivedChains) {
        for (const auto &revision : revisions) {
            checkRevision(revision, /* derived = */ true);
        }
    }

    // The reasons are a set in spirit: one line per feature, not per mention.
    std::sort(out.begin() + long(before), out.end());
    out.erase(std::unique(out.begin() + long(before), out.end()), out.end());
    return out.size() == before;
}

} // namespace rigExec
