// The sealed compiled-program view answers exactly what the running program
// holds: every count, every record by index, the operation graph, the
// declared reads and writes, the digests, the bound paths and the overridden
// inputs. Sharing hands out the same version, never a copy.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/compiledProgram.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecSchema/schemaNames.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/stage.h"
#include <cstdio>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

// Schema names the skinned-mesh fixture below authors, each declared once;
// RigExec spellings stay single-sourced in schemaNames.h.
TF_DEFINE_PRIVATE_TOKENS(
    _sealedViewTokens,
    ((points, "points"))
    ((pointsType, "Points"))
    ((control, rigExec::SchemaTypeName::RigExecControl))
    ((avarsTx, rigExec::SchemaAttrText::avarsTx))
    ((skinMover, rigExec::SchemaTypeName::RigExecSkinMover))
    ((moverAPI, rigExec::SchemaTypeName::RigExecMoverAPI))
    ((moves, rigExec::SchemaAttrText::rigExecMoves))
    ((influences, rigExec::SchemaAttrText::rigExecInfluences))
    ((elementSize, rigExec::SchemaAttrText::rigExecElementSize))
    ((jointIndices, rigExec::SchemaAttrText::rigExecJointIndices))
    ((jointWeights, rigExec::SchemaAttrText::rigExecJointWeights))
    ((staticWeight, rigExec::SchemaTypeName::RigExecStaticWeight))
    ((weightTarget, rigExec::SchemaAttrText::rigExecWeightTarget))
    ((representation, rigExec::SchemaAttrText::rigExecRepresentation))
    ((sparse, rigExec::SchemaValueText::sparse))
    ((indices, rigExec::SchemaAttrText::rigExecIndices))
    ((values, rigExec::SchemaAttrText::rigExecValues))
    ((weightObject, rigExec::SchemaAttrText::rigExecWeightObject))
);

namespace {
int failures = 0;
#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);   \
        }                                                                      \
    } while (0)

bool
Compile(RigExecRigEvaluator &evaluator)
{
    std::vector<std::string> errors;
    if (evaluator.Compile(&errors)) {
        return true;
    }
    for (const auto &error : errors) {
        std::printf("    %s\n", error.c_str());
    }
    return false;
}

UsdStageRefPtr
MakeStage()
{
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/View"), TfToken("Scope"));
    const SdfPath root("/View/Rig");
    stage->DefinePrim(root, TfToken("RigExecRoot"));
    auto channels =
        stage->DefinePrim(SdfPath("/View/Channels"), TfToken("Scope"));
    const auto own = channels.CreateAttribute(TfToken("own"),
                                              SdfValueTypeNames->Double, true);
    CHECK(own.Set(3.0));
    const auto math = [&](const char *name, float argument) {
        auto prim = stage->DefinePrim(root.AppendChild(TfToken(name)),
                                      TfToken("RigExecFloatMathMover"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        CHECK(prim.GetRelationship(TfToken("rigExec:moves"))
                  .SetTargets({own.GetPath()}));
        CHECK(prim.GetAttribute(TfToken("rigExec:operation"))
                  .Set(TfToken("add")));
        CHECK(prim.GetAttribute(TfToken("inputs:value")).Set(argument));
        return prim;
    };
    math("AFirst", 1.0f);
    math("BSecond", 2.0f);

    // A skinned mesh, so the chain, revision and weight loops below have
    // records to compare against rather than running empty.
    const SdfPath geomPath("/View/Geom/Panel");
    const SdfPath pointsProp =
        geomPath.AppendProperty(_sealedViewTokens->points);
    const UsdPrim mesh =
        stage->DefinePrim(geomPath, _sealedViewTokens->pointsType);
    VtVec3fArray panelPoints(4);
    for (size_t i = 0; i < panelPoints.size(); ++i) {
        panelPoints[i] = GfVec3f(float(i), float(i % 2), 0.0f);
    }
    CHECK(mesh.GetAttribute(_sealedViewTokens->points).Set(panelPoints));
    const UsdPrim joint = stage->DefinePrim(SdfPath("/View/Rig/Joint"),
                                            _sealedViewTokens->control);
    CHECK(joint.GetAttribute(_sealedViewTokens->avarsTx).Set(1.0));
    const auto skin = [&](const char *name) {
        UsdPrim prim = stage->DefinePrim(root.AppendChild(TfToken(name)),
                                         _sealedViewTokens->skinMover);
        CHECK(prim.ApplyAPI(_sealedViewTokens->moverAPI));
        CHECK(prim.GetRelationship(_sealedViewTokens->moves)
                  .SetTargets({pointsProp}));
        CHECK(prim.CreateRelationship(_sealedViewTokens->influences)
                  .SetTargets({joint.GetPath()}));
        CHECK(prim.CreateAttribute(_sealedViewTokens->elementSize,
                                   SdfValueTypeNames->Int)
                  .Set(1));
        CHECK(prim.CreateAttribute(_sealedViewTokens->jointIndices,
                                   SdfValueTypeNames->IntArray)
                  .Set(VtIntArray(4, 0)));
        CHECK(prim.CreateAttribute(_sealedViewTokens->jointWeights,
                                   SdfValueTypeNames->FloatArray)
                  .Set(VtFloatArray(4, 1.0f)));
        return prim;
    };
    skin("PanelSkinA");
    const UsdPrim weight =
        stage->DefinePrim(SdfPath("/View/Rig/Weights/PanelEdge"),
                          _sealedViewTokens->staticWeight);
    CHECK(weight.CreateRelationship(_sealedViewTokens->weightTarget, false)
              .SetTargets({pointsProp}));
    CHECK(weight
              .CreateAttribute(_sealedViewTokens->representation,
                               SdfValueTypeNames->Token, false)
              .Set(_sealedViewTokens->sparse));
    CHECK(weight
              .CreateAttribute(_sealedViewTokens->indices,
                               SdfValueTypeNames->IntArray, false)
              .Set(VtIntArray({0, 1, 2, 3})));
    CHECK(weight
              .CreateAttribute(_sealedViewTokens->values,
                               SdfValueTypeNames->FloatArray, false)
              .Set(VtFloatArray(4, 1.0f)));
    CHECK(skin("PanelSkinB")
              .CreateRelationship(_sealedViewTokens->weightObject, false)
              .SetTargets({weight.GetPath()}));
    return stage;
}

bool
SameRanges(const std::vector<RigExecOpSlotRange> &a,
           const std::vector<RigExecOpSlotRange> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].domain != b[i].domain || a[i].first != b[i].first ||
            a[i].last != b[i].last) {
            return false;
        }
    }
    return true;
}

bool
SameGraph(const std::vector<RigExecOpGraphNode> &a,
          const std::vector<RigExecOpGraphNode> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].step != b[i].step || a[i].kind != b[i].kind ||
            a[i].domain != b[i].domain || a[i].label != b[i].label ||
            a[i].preds != b[i].preds || a[i].succs != b[i].succs ||
            a[i].cluster != b[i].cluster ||
            !SameRanges(a[i].reads, b[i].reads) ||
            !SameRanges(a[i].writes, b[i].writes)) {
            return false;
        }
    }
    return true;
}

void
TestView()
{
    UsdStageRefPtr stage = MakeStage();
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/View/Rig"));
    CHECK(Compile(evaluator));
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const RigExecCompiledProgram &V = program->GetCompiledProgram();
    CHECK(B.steps.size() > 0);
    CHECK(B.propertyChains.size() > 0);

    CHECK(V.GetStepCount() == B.steps.size());
    for (size_t i = 0; i < B.steps.size(); ++i) {
        CHECK(&V.GetStep(i) == &B.steps[i]);
        CHECK(&V.GetStepReads(i) == &B.steps[i].reads);
        CHECK(&V.GetStepWrites(i) == &B.steps[i].writes);
    }
    CHECK(V.GetExcludedStepCount() == B.excludedSteps.size());
    for (size_t i = 0; i < B.excludedSteps.size(); ++i) {
        CHECK(&V.GetExcludedStep(i) == &B.excludedSteps[i]);
    }
    CHECK(V.GetCommitCount() == B.commits.size());
    for (size_t i = 0; i < B.commits.size(); ++i) {
        CHECK(&V.GetCommit(i) == &B.commits[i]);
    }
    CHECK(V.GetComposeGroupCount() == B.composeGroups.size());
    for (size_t i = 0; i < B.composeGroups.size(); ++i) {
        CHECK(&V.GetComposeGroup(i) == &B.composeGroups[i]);
    }
    CHECK(V.GetProviderSlotCount() == B.paths.size());
    CHECK(V.GetLadderCount() == B.ladders.size());
    for (size_t i = 0; i < B.ladders.size(); ++i) {
        CHECK(&V.GetLadder(i) == &B.ladders[i]);
    }
    CHECK(V.GetSpaceSwitchCount() == B.spaceSwitches.size());
    for (size_t i = 0; i < B.spaceSwitches.size(); ++i) {
        CHECK(&V.GetSpaceSwitch(i) == &B.spaceSwitches[i]);
    }
    static_assert(std::is_same<decltype(V.GetChainFacts(size_t(0))),
                               const RigExecCompiledChainFacts &>::value);
    static_assert(
        std::is_same<decltype(V.GetChainRevisionFacts(size_t(0), size_t(0))),
                     const RigExecCompiledRevisionFacts &>::value);
    static_assert(std::is_same<decltype(V.GetWeightObjectFacts(size_t(0))),
                               const RigExecCompiledWeightFacts &>::value);

    CHECK(V.GetChainCount() == B.chains.size());
    CHECK(!B.chains.empty());
    for (size_t i = 0; i < V.GetChainCount(); ++i) {
        CHECK(V.GetChainFacts(i).target == B.chains[i].target);
        CHECK(V.GetChainRevisionCount(i) == B.chains[i].revisions.size());
        for (size_t r = 0; r < V.GetChainRevisionCount(i); ++r) {
            const RigExecCompiledRevisionFacts &facts =
                V.GetChainRevisionFacts(i, r);
            CHECK(facts.moverPath == B.chains[i].revisions[r].moverPath);
            CHECK(facts.target == B.chains[i].revisions[r].target);
            CHECK(facts.op == B.chains[i].revisions[r].op);
        }
    }
    CHECK(V.GetRevisionCount() == B.revisionIndex.size());
    for (size_t i = 0; i < B.revisionIndex.size(); ++i) {
        CHECK(V.GetRevisionIdentity(i) == B.revisionIndex[i]);
    }
    CHECK(V.GetDerivedTargetCount() == B.derivedIndex.size());
    for (size_t i = 0; i < B.derivedIndex.size(); ++i) {
        CHECK(V.GetDerivedIdentity(i) == B.derivedIndex[i]);
    }
    CHECK(V.GetWeightObjectCount() == B.weightObjects.size());
    CHECK(!B.weightObjects.empty());
    for (size_t i = 0; i < V.GetWeightObjectCount(); ++i) {
        CHECK(V.GetWeightObjectFacts(i).path == B.weightObjects[i].path);
        CHECK(V.GetWeightObjectFacts(i).type == B.weightObjects[i].type);
    }
    CHECK(V.GetConstraintCount() == B.constraints.size());
    for (size_t i = 0; i < B.constraints.size(); ++i) {
        CHECK(&V.GetConstraint(i) == &B.constraints[i]);
    }
    CHECK(V.GetSolverCount() == B.solvers.size());
    for (size_t i = 0; i < B.solvers.size(); ++i) {
        CHECK(&V.GetSolver(i) == &B.solvers[i]);
    }
    CHECK(V.GetSolverArrayCount() == B.solverArrays.size());
    CHECK(&V.GetSolverArrays() == &B.solverArrays);
    CHECK(V.GetConstraintArrayCount() == B.constraintArrays.size());
    for (size_t i = 0; i < B.constraintArrays.size(); ++i) {
        CHECK(&V.GetConstraintArray(i) == &B.constraintArrays[i]);
    }
    CHECK(V.GetPropertyChainCount() == B.propertyChains.size());
    for (size_t i = 0; i < B.propertyChains.size(); ++i) {
        CHECK(&V.GetPropertyChain(i) == &B.propertyChains[i]);
    }
    CHECK(V.GetPropertyRecordCount() == B.propertyRecords.size());
    for (size_t i = 0; i < B.propertyRecords.size(); ++i) {
        CHECK(&V.GetPropertyRecord(i) == &B.propertyRecords[i]);
    }
    CHECK(V.GetFrameRecordCount() == B.frameRecords.size());
    for (size_t i = 0; i < B.frameRecords.size(); ++i) {
        CHECK(&V.GetFrameRecord(i) == &B.frameRecords[i]);
    }
    CHECK(V.GetPoseInterpolatorCount() == B.poseInterpolators.size());
    for (size_t i = 0; i < B.poseInterpolators.size(); ++i) {
        CHECK(&V.GetPoseInterpolator(i) == &B.poseInterpolators[i]);
    }
    CHECK(V.GetClusterCount() == B.clustering.clusters.size());
    for (size_t i = 0; i < B.clustering.clusters.size(); ++i) {
        CHECK(&V.GetCluster(i) == &B.clustering.clusters[i]);
    }
    CHECK(V.GetPointBindingCount() == size_t(B.pointBindingCount));
    CHECK(V.GetNativeSourceCount() == B.nativeSources.size());
    for (size_t i = 0; i < B.nativeSources.size(); ++i) {
        CHECK(&V.GetNativeSource(i) == &B.nativeSources[i]);
    }
    CHECK(V.GetXformSlotCount() == B.xformSlots.size());
    for (size_t i = 0; i < B.xformSlots.size(); ++i) {
        CHECK(V.GetXformSlot(i) == B.xformSlots[i]);
        CHECK(V.GetXformPrim(i) == B.xformPrimsBySlot[i]);
    }
    CHECK(V.GetReaderWalkCount() == B.readerWalks.size());
    for (size_t i = 0; i < B.readerWalks.size(); ++i) {
        CHECK(&V.GetReaderWalk(i) == &B.readerWalks[i]);
    }

    CHECK(SameGraph(V.GetOpGraph(), RigExecBakedOpGraph(B)));
    CHECK(&V.GetWindowedOps() == &B.windowedOps);
    CHECK(&V.GetCones() == &B.cones);
    CHECK(&V.GetCrossDomainReads() == &B.crossDomainReads);
    CHECK(&V.GetRequiredStageFramesAdmission() ==
          &B.requiredStageFramesAdmission);
    CHECK(&V.GetDeltaBasePaths() == &B.deltaBasePaths);

    const uint64_t signature = uint64_t(evaluator.GetCompiledRigSignature());
    CHECK(V.GetProgramDigest(signature) ==
          (signature ^ (uint64_t(B.boundInputs) * 1099511628211ull) ^
           (uint64_t(B.varyingInputs) * 16777619ull)));
    CHECK(V.GetConstantDigest() == RigExecFrozenAvarRegionDigest(*program));
    CHECK(&V.GetAvarConstants() == &B.avarConstants);
    CHECK(&V.GetAvarBindings() == &B.avarBindings);
    CHECK(&V.GetAvarConstantBindings() == &B.avarConstantBindings);
    CHECK(&V.GetPromotedAvars() == &B.promotedAvars);
    CHECK(V.GetAvarConstantCount() == B.avarConstants.size());
    CHECK(V.GetAvarConstantBindingCount() == B.avarConstantBindings.size());
    CHECK(V.GetAvarBindingCount() == B.avarBindings.size());
    CHECK(V.GetAvarSlotCount() == B.avars.size());

    CHECK(&V.GetBoundPaths() == &B.paths);
    if (B.pathTexts) {
        CHECK(&V.GetPathTexts() == B.pathTexts.get());
    } else {
        CHECK(V.GetPathTexts().empty());
    }
    CHECK(V.GetAssetRootPath() == B.assetRootPath);
    CHECK(&V.GetJointPaths() == &B.jointPaths);
    CHECK(&V.GetControlPaths() == &B.controlPaths);
    CHECK(&V.GetPoseWeightPaths() == &B.poseWeightPaths);
    CHECK(&V.GetChainTargets() == &B.chainTargets);

    CHECK(V.GetBoundInputCount() == B.boundInputs);
    CHECK(V.GetVaryingInputCount() == B.varyingInputs);
    CHECK(&V.GetOverridden() == &B.overridden);
    CHECK(&V.GetOverridableInputs() == &B.overridableInputs);
    CHECK(&V.GetHeadOverrideSlots() == &B.headOverrideSlots);
    CHECK(&V.GetFalloffLuts() == &B.falloffLuts);
    CHECK(&V.GetOraclePublications() == &B.oraclePublications);
    CHECK(V.GetJointSolverBinding() == B.jointSolverBinding);

    const std::shared_ptr<const RigExecCompiledProgram> first =
        program->ShareCompiledProgram();
    const std::shared_ptr<const RigExecCompiledProgram> second =
        program->ShareCompiledProgram();
    CHECK(first && second);
    CHECK(first == second);
    CHECK(first.get() == &V);
    CHECK(first->IsSameVersionAs(*second));
    CHECK(first.use_count() >= 3);

    std::vector<RigExecBakedEnvelopeObject> implObjects, viewObjects;
    std::map<SdfPath, int> implIndex, viewIndex;
    std::string implError, viewError;
    const bool implOk = RigExecBakedProgram::ComposeEnvelopeObjects(
        evaluator, B, &implObjects, &implIndex, &implError);
    const bool viewOk = RigExecBakedProgram::ComposeEnvelopeObjects(
        evaluator, V, &viewObjects, &viewIndex, &viewError);
    CHECK(implOk == viewOk);
    CHECK(implError == viewError);
    CHECK(implIndex == viewIndex);
    CHECK(implObjects.size() == viewObjects.size());
    for (size_t i = 0; i < implObjects.size() && i < viewObjects.size(); ++i) {
        CHECK(implObjects[i].path == viewObjects[i].path);
        CHECK(implObjects[i].values == viewObjects[i].values);
        CHECK(implObjects[i].indices == viewObjects[i].indices);
        CHECK(implObjects[i].base == viewObjects[i].base);
        CHECK(implObjects[i].inputs == viewObjects[i].inputs);
    }
}

} // namespace

int
main()
{
    // The codeless rigExec schema is a plugin: register it rather than rely on
    // an ambient PXR_PLUGINPATH_NAME.
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestView();
    if (failures == 0) {
        std::printf("testRigExecCompiledProgram passed\n");
    }
    return failures == 0 ? 0 : 1;
}
