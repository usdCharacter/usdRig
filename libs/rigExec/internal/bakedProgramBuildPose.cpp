// Building a baked program: pose descriptors, publication, pose
// interpolators and geometry chains.
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
    ((rigExecIkValue, SchemaAttrText::rigExecIkValue))
    ((inputsGain, SchemaAttrText::inputsGain))
    ((rigExecBasis, SchemaAttrText::rigExecBasis))
    ((rigExecPoseRotations, SchemaAttrText::rigExecPoseRotations))
    ((rigExecPoseFalloffs, SchemaAttrText::rigExecPoseFalloffs))
    ((rigExecPoseGains, SchemaAttrText::rigExecPoseGains))
    ((rigExecKernel, SchemaAttrText::rigExecKernel))
    ((rigExecRegularization, SchemaAttrText::rigExecRegularization))
    ((rigExecTarget, SchemaAttrText::rigExecTarget))
    ((inputsActiveSpace, SchemaAttrText::inputsActiveSpace))
    ((inputsEnabled, SchemaAttrText::inputsEnabled))
    ((inputsMask, SchemaAttrText::inputsMask))
);

} // namespace

bool
RigExecBakedProgram::_SnapshotAfter(const RigExecRigEvaluator &E,
                                    const SdfPath &target,
                                    const SdfPath &writer)
{
    const auto wanted = E._nativePhaseCheckpoints.find(target);
    return wanted != E._nativePhaseCheckpoints.end() &&
           wanted->second.count(writer) > 0;
}

void
RigExecBakedProgram::_BuildPoseDescriptors(_BuildAttempt &build)
{
    auto &excludedPoseWriters = build.excludedPoseWriters;
    RigExecRigEvaluator &E = build.E;
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    // Every writer owns a pose version. Matrix checkpoints expose only the
    // named pairs requested by consumers, preserving their skipped-record fallback.
    auto &walk = build.walk;
    struct PoseDomainEntry {
        bool solverBatch;
        size_t index;
        SdfPath path;
    };
    std::vector<PoseDomainEntry> domains;
    for (size_t i = 0; i < E._solverBatches.size(); ++i) {
        const auto &batch = E._solverBatches[i];
        if (!batch.solvers.empty())
            domains.push_back({true, i, *batch.solvers.begin()});
    }
    for (size_t i = 0; i < E._frameConstraints.size(); ++i)
        domains.push_back({false, i, E._frameConstraints[i].moverPath});
    B.poseDependencyPaths = E._nativePoseDependencies;
    std::sort(
        domains.begin(), domains.end(), [&](const auto &a, const auto &b) {
            const auto ai = E._nativePoseOrdinals.find(a.path),
                       bi = E._nativePoseOrdinals.find(b.path);
            const int oa = ai == E._nativePoseOrdinals.end() ? -1 : ai->second;
            const int ob = bi == E._nativePoseOrdinals.end() ? -1 : bi->second;
            return oa != ob ? oa < ob : a.path < b.path;
        });
    for (const PoseDomainEntry &step : domains) {
        RigExecBakedWalkEntry entry;
        entry.solverBatch = step.solverBatch;
        if (step.solverBatch) {
            const auto &batch = E._solverBatches[step.index];
            if (std::any_of(batch.solvers.begin(), batch.solvers.end(),
                            [&](const SdfPath &owner) {
                                return excludedPoseWriters.count(owner) != 0;
                            }))
                continue;
            entry.level = 0;
            for (const SdfPath &solverPath : batch.solvers) {
                entry.batchSolvers.push_back(solverPath);
                const auto joints = E._solverJoints.find(solverPath);
                entry.solverJoints.push_back(
                    joints == E._solverJoints.end()
                        ? std::vector<std::pair<SdfPath, int>>()
                        : joints->second);
                std::vector<char> snapshotJoints;
                for (const auto &[joint, element] : entry.solverJoints.back()) {
                    snapshotJoints.push_back(
                        _SnapshotAfter(E, joint, solverPath) ? 1 : 0);
                }
                entry.solverSnapshotJoints.push_back(std::move(snapshotJoints));
                // The live-rest half of the same binding. A restInputs entry
                // with an EMPTY predecessor is a joint whose
                // authored rest was merely pinned, not a live one.
                SdfPathVector live;
                for (const auto &[joint, predecessor] : batch.restInputs) {
                    if (predecessor.IsEmpty())
                        continue;
                    bool named = false;
                    if (joints != E._solverJoints.end()) {
                        for (const auto &[named_, element] : joints->second) {
                            named = named || named_ == joint;
                        }
                    }
                    if (named)
                        live.push_back(joint);
                }
                entry.solverLiveRestJoints.push_back(std::move(live));
            }
        } else {
            const RigExecRigEvaluator::_FrameConstraint &fc =
                E._frameConstraints[step.index];
            if (excludedPoseWriters.count(fc.moverPath))
                continue;
            entry.constraint.moverPath = fc.moverPath;
            entry.constraint.schemaType = fc.schemaType;
            entry.constraint.targets = fc.targets;
            for (const SdfPath &target : fc.targets) {
                entry.constraint.snapshotTargets.push_back(
                    _SnapshotAfter(E, target, fc.moverPath) ? 1 : 0);
            }
            entry.constraint.pointsTarget = fc.pointsTarget;
            entry.constraint.ikChain = fc.ikChain;
            entry.constraint.ikRestLive = fc.ikRestLive;
            entry.constraint.ikUsesAnimatedTs =
                !fc.ikChain.empty() && E._IkUsesAnimatedTs(fc.ikChain);
            entry.constraint.effector = fc.effector.sourcePath;
            entry.constraint.effectorXform = fc.effector.xformPath;
            for (const auto &pole : fc.poleObjects) {
                entry.constraint.poleObjects.push_back(pole.sourcePath);
                entry.constraint.poleObjectXforms.push_back(pole.xformPath);
            }
            for (const auto &source : fc.sources) {
                entry.constraint.sources.push_back(source.sourcePath);
                entry.constraint.sourceXforms.push_back(source.xformPath);
            }
            entry.constraint.worldUpObject = fc.worldUpObject.sourcePath;
            entry.constraint.weightObject = fc.weightObject;
            entry.constraint.worldUpXform = fc.worldUpObject.xformPath;
            entry.constraint.space = fc.spacePath;
            entry.constraint.blendShear = fc.blendShear;
            entry.constraint.worldUpRotationOnly = fc.worldUpRotationOnly;
            entry.constraint.radialBlend = fc.radialBlend;
        }
        walk.push_back(std::move(entry));
    }
    // Capture aggregate solvers with no authored joint application once.
    // Their explicit aggregate reads join the same canonical graph as every
    // pose and geometry descriptor; discovery order supplies no schedule.
    {
        std::set<SdfPath> batched;
        for (const auto &entry : walk)
            if (entry.solverBatch) {
                for (const SdfPath &solverPath : entry.batchSolvers) {
                    batched.insert(solverPath);
                }
            }
        std::set<SdfPath> pending;
        for (const SdfPath &solverPath : E._nativeGuideSolvers) {
            if (!batched.count(solverPath)) {
                pending.insert(solverPath);
            }
        }
        ctx.guideOnlySolvers.assign(pending.begin(), pending.end());
    }
    RigExecBakedBuildWalk(&ctx, walk);
    // The native sources the walk registered, and the two buffers the
    // prologue fills and the cone compares. Sized here, once, and never
    // resized in a run.
    B.deltaValues.assign(B.deltaBasePaths.size(), GfMatrix4d(1.0));
    B.deltaPresent.assign(B.deltaBasePaths.size(), 0);
    B.deltaBaseMatrix.assign(B.deltaBasePaths.size(), GfMatrix4d(1.0));
    B.lastDeltaBaseMatrix = B.deltaBaseMatrix;
    B.deltaBaseOk.assign(B.deltaBasePaths.size(), 0);
    B.lastDeltaBaseOk = B.deltaBaseOk;
    B.nativeFrames.assign(B.nativeSources.size(), RigExecPointFrame());
    B.lastNativeFrames = B.nativeFrames;
    B.nativeFrameOk.assign(B.nativeSources.size(), 0);
    B.lastNativeFrameOk = B.nativeFrameOk;
}

void
RigExecBakedProgram::_BuildPublication(_BuildAttempt &build)
{
    RigExecRigEvaluator &E = build.E;
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    for (const SdfPath &joint : E._jointPaths) {
        B.jointSlots.push_back(ctx.SlotOf(joint));
        B.jointPaths.push_back(joint);
    }
    for (const SdfPath &control : E._controlPaths) {
        B.controlSlots.push_back(ctx.SlotOf(control));
        B.controlPaths.push_back(control);
    }
    for (const SdfPath &solverPath : E._nativeGuideSolvers) {
        const auto it = B.solverIndex.find(solverPath);
        if (it == B.solverIndex.end()) {
            // Walk + guide pass cover this map exactly.
            ctx.Refuse("solver publishes guides but was not baked", solverPath);
            continue;
        }
        B.solverArrays.emplace_back(solverPath, it->second);
    }
    // Sorted once, here, so the epilogue can fill each published map from
    // its end instead of searching it for every key. The lists themselves
    // are NOT in path order -- the biped's 252 joints and 74 controls both
    // arrive in binding order -- so the permutation is the whole point.
    const auto publishOrder = [](const std::vector<SdfPath> &paths,
                                 std::vector<int> *order) {
        order->resize(paths.size());
        std::iota(order->begin(), order->end(), 0);
        // STABLE, because of the fallback below: when a list does name one
        // path twice the permutation is still what both fill loops walk,
        // and only a stable sort leaves two equal paths in the order the
        // publication list had them -- which is what makes the assigning
        // fallback publish the same value the assignment it replaces did.
        std::stable_sort(order->begin(), order->end(), [&](int a, int b) {
            return paths[size_t(a)] < paths[size_t(b)];
        });
        // Strictly ascending, not merely sorted: a repeated path would make
        // an emplace keep the first value where an assignment keeps the last,
        // so a list that names one twice is published by assignment.
        for (size_t k = 1; k < order->size(); ++k) {
            if (!(paths[size_t((*order)[k - 1])] <
                  paths[size_t((*order)[k])])) {
                return false;
            }
        }
        return true;
    };
    B.jointPathsAscending = publishOrder(B.jointPaths, &B.jointPublishOrder);
    B.controlPathsAscending =
        publishOrder(B.controlPaths, &B.controlPublishOrder);
    std::vector<SdfPath> solverPaths;
    solverPaths.reserve(B.solverArrays.size());
    for (const auto &entry : B.solverArrays) {
        solverPaths.push_back(entry.first);
    }
    B.solverArraysAscending = publishOrder(solverPaths, &B.solverPublishOrder);
    // Sized at Build so the epilogue's two passes allocate nothing.
    B.jointMatrixPublished.assign(B.jointPaths.size(), 0);
}

bool
RigExecBakedProgram::_BuildPoseInterpolators(_BuildAttempt &build)
{
    RigExecRigEvaluator &E = build.E;
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    // Geometry readers bind these compiled weight output identities.
    RigExecPathText interpolatorText;
    for (const RigExecRigEvaluator::_PoseInterpolator &record :
         E._poseInterpolators) {
        if (!record.descriptor) {
            ctx.Refuse("pose interpolator has no detached descriptor",
                       record.prim);
            continue;
        }
        const auto &captured = *record.descriptor;
        RigExecBakedProgramImpl::PoseInterpolator out;
        static_cast<RigExecPoseInterpolatorRecord &>(out) = captured.record;
        out.path = record.prim;
        out.pathText = interpolatorText(record.prim);
        // A numeric driver reads dials, not a frame: it names no driver prim
        // and needs no slot.
        const bool numeric = !captured.driverAttributes.empty();
        const bool frames = captured.ml ? !captured.driver.IsEmpty() : !numeric;
        out.driverSlot = frames ? ctx.SlotOf(captured.driver) : -1;
        if (frames && out.driverSlot < 0) {
            ctx.Refuse("pose interpolator driver has no provider slot",
                       captured.driver);
            continue;
        }
        if (!captured.parent.IsEmpty()) {
            out.parentSlot = ctx.SlotOf(captured.parent);
            if (out.parentSlot < 0) {
                ctx.Refuse("pose interpolator driver parent has no provider "
                           "slot",
                           captured.parent);
                continue;
            }
        }
        bool extraSlots = true;
        for (size_t k = 0; k < captured.extraDrivers.size(); ++k) {
            const int driver = ctx.SlotOf(captured.extraDrivers[k]);
            const int parent = captured.extraParents[k].IsEmpty()
                                   ? -1
                                   : ctx.SlotOf(captured.extraParents[k]);
            if (driver < 0 ||
                (parent < 0 && !captured.extraParents[k].IsEmpty())) {
                ctx.Refuse("pose interpolator driver has no provider slot",
                           captured.extraDrivers[k]);
                extraSlots = false;
                break;
            }
            out.extraDriverSlots.push_back(driver);
            out.extraParentSlots.push_back(parent);
        }
        if (!extraSlots) {
            continue;
        }
        const UsdPrim prim = B.stage->GetPrimAtPath(record.prim);
        // The interpolator's structure -- its driver, its poses, their
        // rotations and radii -- is compiled rig identity and recompiles; what a
        // frame reads is the one enable, bound like any other input so an
        // animated or dragged one is honoured and a value edit needs no
        // rebuild.
        B.prims.insert(record.prim);
        B.resolvedRoutedPrims.insert(record.prim);
        out.enabled = ctx.Bind<bool>(prim, _tokens->inputsEnabled, true);
        if (captured.ml) {
            out.mask = ctx.Bind<float>(prim, _tokens->inputsMask, 1.0f);
        }
        for (size_t k = 0; k < captured.driverAttributes.size(); ++k) {
            const SdfPath &value = captured.driverAttributes[k];
            const UsdPrim owner = B.stage->GetPrimAtPath(value.GetPrimPath());
            // NAMED, for invalidation, but NOT routed: a routed prim tells
            // SetOverrides that every read of it already goes through the
            // resolved inputs, and a drag on the dial would then be placed
            // nowhere -- the bound input below is what reads it, and it is
            // reached through its own override index.
            B.prims.insert(value.GetPrimPath());
            const TfToken &valueName = value.GetNameToken();
            const bool isFloat = k < captured.numericIsFloat.size() &&
                                 captured.numericIsFloat[k];
            out.valueIsFloat.push_back(isFloat ? 1 : 0);
            out.valueInputs.push_back(
                isFloat ? RigExecBakedInput<double>()
                        : ctx.Bind<double>(owner, valueName, 0.0));
            out.floatValueInputs.push_back(
                isFloat ? ctx.Bind<float>(owner, valueName, 0.0f)
                        : RigExecBakedInput<float>());
        }
        out.values.assign(out.valueInputs.size(), 0.0);
        const auto addWeight = [&B](const SdfPath &weight) {
            const int slot = int(B.poseWeightPaths.size());
            B.poseWeightPaths.push_back(weight);
            B.poseWeightIndex[weight] = slot;
            return slot;
        };
        out.weightBegin = int(B.poseWeightPaths.size());
        for (const SdfPath &weight : captured.disabledPoseWeights) {
            out.disabledSlots.push_back(addWeight(weight));
        }
        for (const SdfPath &weight : captured.poseWeights) {
            out.poseSlots.push_back(addWeight(weight));
        }
        for (size_t k = 0; k < captured.primvarPaths.size(); ++k) {
            out.primvarSlots.push_back(addWeight(captured.primvarPaths[k]));
            out.primvarSources.push_back(int(captured.primvarSources[k]));
        }
        out.weightEnd = int(B.poseWeightPaths.size());
        out.poseCount = out.poseSlots.size();
        out.kernelInputs.numeric.resize(out.valueInputs.size());
        out.kernelInputs.extraDrivers.resize(out.extraDriverSlots.size());
        B.poseInterpolators.push_back(std::move(out));
    }
    B.poseWeights.assign(B.poseWeightPaths.size(), 0.0f);

    // ---- space switches ------------------------------------------------------
    //
    // Detached switch structure supplies ordered sources and masks; the
    // common graph binds the active selector and every frame producer.
    B.spaceSwitchBySlot.assign(B.paths.size(), -1);
    for (const RigExecRigEvaluator::_SpaceSwitch &sw : E._spaceSwitches) {
        if (!sw.descriptor) {
            ctx.Refuse("space switch has no detached descriptor",
                       sw.switchPath);
            continue;
        }
        const auto &captured = *sw.descriptor;
        RigExecBakedProgramImpl::SpaceSwitch out;
        out.kernelRecord = captured.record;
        out.slot = ctx.SlotOf(captured.target);
        if (out.slot < 0)
            continue;
        for (size_t k = 0; k < captured.sources.size(); ++k) {
            const auto &source = captured.sources[k];
            out.sourceSlots.push_back(source.IsEmpty() ? -1
                                                       : ctx.SlotOf(source));
            out.filters.push_back(
                RigExecRotationFilter(captured.record.filters[k]));
        }
        out.twistAxis = captured.record.twistAxis;
        out.spaceSlot =
            captured.space.IsEmpty() ? -1 : ctx.SlotOf(captured.space);
        for (size_t axis = 0; axis < 3; ++axis) {
            out.affectTranslation[axis] =
                captured.record.affectTranslation[axis];
            out.affectRotation[axis] = captured.record.affectRotation[axis];
            out.affectScale[axis] = captured.record.affectScale[axis];
        }
        out.labels = captured.labels;
        const UsdPrim switchPrim = B.stage->GetPrimAtPath(sw.switchPath);
        const UsdAttribute active =
            captured.activeAttribute.IsEmpty()
                ? switchPrim.GetAttribute(_tokens->inputsActiveSpace)
                : B.stage->GetAttributeAtPath(captured.activeAttribute);
        if (active && active.GetTypeName() == SdfValueTypeNames->Token) {
            out.tokenIndex = true;
            out.activeTokenInput = ctx.Bind<TfToken>(
                active.GetPrim(), active.GetName(), TfToken());
        } else if (active) {
            out.activeInput = ctx.Bind<double>(
                active.GetPrim(), active.GetName(), captured.activeFallback);
        }
        out.kernelInputs.sources.resize(out.sourceSlots.size());
        B.spaceSwitchBySlot[size_t(out.slot)] = int(B.spaceSwitches.size());
        B.spaceSwitches.push_back(std::move(out));
    }
    // An IK/FK match re-runs its pole's space with the IK control moved:
    // which sources ride the IK control is namespace topology, settled here.
    for (auto &c : B.constraints) {
        if (!c.ikFkMatch || c.sources.size() != 10 || c.sources[5] < 0 ||
            c.sources[6] < 0)
            continue;
        const SdfPath &ik = B.paths[size_t(c.sources[5])];
        const int pole = c.sources[6];
        c.matchPoleSwitch = B.spaceSwitchBySlot[size_t(pole)];
        c.matchPoleSourceRides.clear();
        if (c.matchPoleSwitch >= 0) {
            for (const int source :
                 B.spaceSwitches[size_t(c.matchPoleSwitch)].sourceSlots) {
                c.matchPoleSourceRides.push_back(
                    source >= 0 && B.paths[size_t(source)].HasPrefix(ik) ? 1
                                                                         : 0);
            }
            c.matchPoleInputs.sources.reserve(c.matchPoleSourceRides.size());
        }
        const int parent = B.parent[size_t(pole)];
        c.matchPoleParentRides =
            parent >= 0 && B.paths[size_t(parent)].HasPrefix(ik);
    }
    B.autoClavicleBySlot.assign(B.paths.size(), -1);
    for (const auto &[path, node] : B.sceneDescriptors->nodes) {
        if (node.domain != RigExecSceneDomain::AutoClavicle ||
            !node.fact.active || !path.HasPrefix(E._rigPath))
            continue;
        RigExecSceneCompileInputs source(*B.sceneDescriptors);
        const auto prim = B.stage->GetPrimAtPath(path);
        for (const TfToken &name :
             {_tokens->rigExecIkValue, _tokens->inputsGain,
              _tokens->rigExecBasis, _tokens->rigExecPoseRotations,
              _tokens->rigExecPoseFalloffs, _tokens->rigExecPoseGains,
              _tokens->rigExecKernel, _tokens->rigExecRegularization})
            ctx.Fold(prim, name);
        const auto targets =
            source.Targets(path.AppendProperty(_tokens->rigExecTarget));
        if (targets.size() != 1 || ctx.SlotOf(targets.front()) < 0) {
            ctx.Refuse("auto clavicle requires one provider target", path);
            return false;
        }
        RigExecBakedProgramImpl::AutoClavicle ac;
        ac.slot = ctx.SlotOf(targets.front());
        if (B.autoClavicleBySlot[size_t(ac.slot)] >= 0) {
            ctx.Refuse("multiple auto clavicles target the same provider",
                       path);
            return false;
        }
        RigExecValueId next = 1;
        RigExecSceneGraphBindingContext link;
        // resolve's required id is a reference; the reason is optional and
        // only filled when asked for.
        link.resolve = [&](const RigExecSceneGraphReadRequest &request,
                           RigExecValueId &id, std::string *why) {
            id = next++;
            if (request.domain == RigExecSceneValueDomain::Pose) {
                const int slot = ctx.SlotOf(request.source);
                if (slot < 0) {
                    if (why)
                        *why = "missing auto clavicle frame: " +
                               request.source.GetString();
                    return false;
                }
                RigExecBakedProgramImpl::AutoClavicle::FrameRead read;
                read.value = id;
                read.slot = slot;
                read.computation = request.computation;
                if (read.computation.empty() &&
                    request.source.HasPrefix(targets.front())) {
                    for (int at = slot; at != ac.slot;
                         at = B.parent[size_t(at)]) {
                        if (at < 0) {
                            if (why)
                                *why = "auto clavicle descendant has no target "
                                       "ancestor";
                            return false;
                        }
                        if (B.spaceSwitchBySlot[size_t(at)] >= 0) {
                            if (why)
                                *why = "auto clavicle entering descendant has "
                                       "an independent space switch";
                            return false;
                        }
                        read.recompose.push_back(at);
                    }
                    std::reverse(read.recompose.begin(), read.recompose.end());
                }
                ac.frames.push_back(std::move(read));
                return true;
            }
            const auto attribute = B.stage->GetAttributeAtPath(request.source);
            if (!attribute)
                return false;
            RigExecBakedProgramImpl::AutoClavicle::ScalarRead read;
            read.value = id;
            read.isFloat = attribute.GetTypeName() == SdfValueTypeNames->Float;
            if (read.isFloat)
                read.narrow = ctx.Bind<float>(attribute.GetPrim(),
                                              attribute.GetName(), 0.0f);
            else
                read.wide = ctx.Bind<double>(attribute.GetPrim(),
                                             attribute.GetName(), 0.0);
            ac.scalars.push_back(std::move(read));
            return true;
        };
        std::string invalid;
        if (!RigExecDispatchBindAutoClavicle(*B.sceneDescriptors, path, link, 0,
                                             UINT64_MAX, &ac.operation,
                                             &invalid)) {
            ctx.Refuse(invalid, path);
            return false;
        }
        ac.operation.output = next++;
        ac.values = RigExecTypedValueStore(size_t(next));
        B.autoClavicleBySlot[size_t(ac.slot)] = int(B.autoClavicles.size());
        B.autoClavicles.push_back(std::move(ac));
    }
    // Once every switched slot is known: which version of each frame a
    // switch reads depends on which controls above it are switched.
    {
        std::string invalid;
        if (!RigExecBakedBindSpaceSwitchVersions(&B, &invalid)) {
            ctx.Refuse(invalid, E._rigPath);
            return false;
        }
    }
    return true;
}

void
RigExecBakedProgram::_BuildGeometry(_BuildAttempt &build)
{
    auto &excludedSolverBodies = build.excludedSolverBodies;
    RigExecRigEvaluator &E = build.E;
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    std::vector<RigExecBakedChainSpec> chainSpecs;
    const auto revisionSpec =
        [&E](const RigExecRigEvaluator::_GraphRevision &r) {
            RigExecBakedRevisionSpec spec;
            spec.moverPath = r.moverPath;
            spec.target = r.target;
            spec.op = r.op;
            spec.binding = r.binding;
            spec.transformFinalPhase = r.transformFinalPhase;
            spec.transformPosedPoints = r.transformPosedPoints;
            spec.skinTopologyFixed = r.skinTopologyFixed;
            spec.snapshotAfter = _SnapshotAfter(E, r.target, r.moverPath);
            return spec;
        };
    const auto keepRevision =
        [&](const RigExecRigEvaluator::_GraphRevision &revision) {
            if (!excludedSolverBodies.count(revision.binding.driverFrames))
                return true;
            // The reason is joined into a multi-line note the report prints
            // long after Build, so the map holds the text it was built from.
            const std::string reason =
                "driver frames " + revision.binding.driverFrames.GetString() +
                " is not a solver of this rig; mover set aside";
            const auto added =
                B.cycleSkipReasons.emplace(revision.moverPath, reason);
            if (!added.second &&
                added.first->second.find(reason) == std::string::npos)
                added.first->second += "\n" + reason;
            return false;
        };
    for (const auto &[target, nativeRevisions] : E._graphChains) {
        const auto chainIt = E._graphChains.find(target);
        if (chainIt == E._graphChains.end())
            continue;
        RigExecBakedChainSpec spec;
        spec.target = target;
        for (const auto &revision : chainIt->second) {
            if (keepRevision(revision))
                spec.revisions.push_back(revisionSpec(revision));
        }
        const auto derivedIt = E._graphDerivedChains.find(target);
        if (derivedIt != E._graphDerivedChains.end()) {
            for (const auto &derived : derivedIt->second) {
                if (keepRevision(derived))
                    spec.derived.push_back(revisionSpec(derived));
            }
        }
        chainSpecs.push_back(std::move(spec));
    }
    RigExecBakedBuildGeometry(&ctx, chainSpecs);
}

} // namespace rigExec
