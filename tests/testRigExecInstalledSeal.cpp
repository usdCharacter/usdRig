// A consumer built against the installed SDK alone: the sealed compiled-program
// view answers the queries it advertises, and the evaluator's execution records
// are not nameable from the installed headers. Each probe is a pair of
// overloads; the first is viable only when the name is reachable through the
// installed headers, so a probe returning 1 is a record the seal keeps out.
#include "rigExec/compiledProgram.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecSchema/schemaTokens.h"

// The install ships the sealed view, not the evaluator's execution records.
// This file is built only against the installed SDK, so these headers must not
// be reachable here.
#if __has_include("rigExec/bakedProgramImpl.h")
#error "bakedProgramImpl.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/frozenProgram.h")
#error "frozenProgram.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/frozenContextInternal.h")
#error "frozenContextInternal.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/bakedSchedule.h")
#error "bakedSchedule.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/bakedOpGraph.h")
#error "bakedOpGraph.h is shipped to installed consumers"
#endif

#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/points.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

TF_DEFINE_PRIVATE_TOKENS(
    _sealTokens,
    ((control, rigExec::SchemaTypeName::RigExecControl))
    ((avarsTx, rigExec::SchemaAttrText::avarsTx))
    ((skinMover, rigExec::SchemaTypeName::RigExecSkinMover))
    ((moverAPI, rigExec::SchemaTypeName::RigExecMoverAPI))
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

template <class View>
int
_WritesSolverScratch(
    View &view,
    decltype(view.GetSolver(0).ribbonPointsBinding.gather.clear(), 0))
{
    return 0;
}
template <class View>
long
_WritesSolverScratch(const View &view, long)
{
    return 1;
}

// Even a read: the pose interpolator's solve output is builder scratch.
template <class View>
int
_ReadsInterpolatorScratch(const View &view,
                          decltype(view.GetPoseInterpolator(0).scratch, 0))
{
    return 0;
}
template <class View>
long
_ReadsInterpolatorScratch(const View &view, long)
{
    return 1;
}

// Each pair names one record the view used to lend by index; the first
// overload is viable only when that accessor still exists.
template <class View>
int
_LendsChainRecord(View &view, decltype(view.GetChain(0), 0))
{
    return 0;
}
template <class View>
long
_LendsChainRecord(const View &view, long)
{
    return 1;
}

template <class View>
int
_LendsRevisionRecord(View &view, decltype(view.GetChainRevision(0, 0), 0))
{
    return 0;
}
template <class View>
long
_LendsRevisionRecord(const View &view, long)
{
    return 1;
}

template <class View>
int
_LendsWeightRecord(View &view, decltype(view.GetWeightObject(0), 0))
{
    return 0;
}
template <class View>
long
_LendsWeightRecord(const View &view, long)
{
    return 1;
}

template <class Program>
int
_ReachesStepGraph(Program &program, decltype(program.GetStepGraph(), 0))
{
    return 0;
}
template <class Program>
long
_ReachesStepGraph(const Program &program, long)
{
    return 1;
}

template <class Handle>
int
_ReadsFrozenHandle(Handle &handle, decltype(handle->program.get(), 0))
{
    return 0;
}
template <class Handle>
long
_ReadsFrozenHandle(const Handle &handle, long)
{
    return 1;
}

template <class Impl>
int
_NamesImplType(int, decltype(sizeof(Impl), 0))
{
    return 0;
}
template <class Impl>
long
_NamesImplType(long, long)
{
    return 1;
}

UsdStageRefPtr
MakeStage()
{
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/View"), UsdGeomTokens->Scope);
    const SdfPath root("/View/Rig");
    stage->DefinePrim(root, SchemaType::RigExecRoot);
    const UsdPrim channels =
        stage->DefinePrim(SdfPath("/View/Channels"), UsdGeomTokens->Scope);
    const auto dial = channels.CreateAttribute(TfToken("dial"),
                                               SdfValueTypeNames->Double, true);
    CHECK(dial.Set(3.0));
    const auto math = [&](const char *name, float argument) {
        UsdPrim prim = stage->DefinePrim(root.AppendChild(TfToken(name)),
                                         SchemaType::RigExecFloatMathMover);
        CHECK(prim.ApplyAPI(SchemaType::RigExecMoverAPI));
        CHECK(prim.GetRelationship(SchemaAttr::rigExecMoves)
                  .SetTargets({dial.GetPath()}));
        CHECK(prim.GetAttribute(SchemaAttr::rigExecOperation)
                  .Set(TfToken(SchemaValueText::add)));
        CHECK(prim.GetAttribute(SchemaAttr::inputsValue).Set(argument));
        return prim;
    };
    math("AFirst", 1.0f);
    math("BSecond", 2.0f);

    const SdfPath geomPath("/View/Geom/Panel");
    const UsdPrim mesh = UsdGeomPoints::Define(stage, geomPath).GetPrim();
    const SdfPath pointsProp = geomPath.AppendProperty(UsdGeomTokens->points);
    VtVec3fArray panelPoints(4);
    for (size_t i = 0; i < panelPoints.size(); ++i) {
        panelPoints[i] = GfVec3f(float(i), float(i % 2), 0.0f);
    }
    CHECK(mesh.GetAttribute(UsdGeomTokens->points).Set(panelPoints));
    const UsdPrim joint =
        stage->DefinePrim(SdfPath("/View/Rig/Joint"), _sealTokens->control);
    CHECK(joint.GetAttribute(_sealTokens->avarsTx).Set(1.0));
    UsdPrim skin = stage->DefinePrim(root.AppendChild(TfToken("PanelSkinA")),
                                     _sealTokens->skinMover);
    CHECK(skin.ApplyAPI(_sealTokens->moverAPI));
    CHECK(skin.GetRelationship(SchemaAttr::rigExecMoves)
              .SetTargets({pointsProp}));
    CHECK(skin.CreateRelationship(_sealTokens->influences)
              .SetTargets({joint.GetPath()}));
    CHECK(skin.CreateAttribute(_sealTokens->elementSize, SdfValueTypeNames->Int)
              .Set(1));
    CHECK(skin.CreateAttribute(_sealTokens->jointIndices,
                               SdfValueTypeNames->IntArray)
              .Set(VtIntArray(4, 0)));
    CHECK(skin.CreateAttribute(_sealTokens->jointWeights,
                               SdfValueTypeNames->FloatArray)
              .Set(VtFloatArray(4, 1.0f)));
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/View/Rig/Weights/PanelEdge"), _sealTokens->staticWeight);
    CHECK(weight.CreateRelationship(_sealTokens->weightTarget, false)
              .SetTargets({pointsProp}));
    CHECK(weight
              .CreateAttribute(_sealTokens->representation,
                               SdfValueTypeNames->Token, false)
              .Set(_sealTokens->sparse));
    CHECK(weight
              .CreateAttribute(_sealTokens->indices,
                               SdfValueTypeNames->IntArray, false)
              .Set(VtIntArray({0, 1, 2, 3})));
    CHECK(weight
              .CreateAttribute(_sealTokens->values,
                               SdfValueTypeNames->FloatArray, false)
              .Set(VtFloatArray(4, 1.0f)));
    CHECK(skin.CreateRelationship(_sealTokens->weightObject, false)
              .SetTargets({weight.GetPath()}));
    return stage;
}

void
TestSeal()
{
    UsdStageRefPtr stage = MakeStage();
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/View/Rig"));
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const auto &error : errors) {
            std::printf("compile: %s\n", error.c_str());
        }
        ++failures;
        return;
    }
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program);
    if (!program) {
        return;
    }
    const RigExecCompiledProgram &view = program->GetCompiledProgram();

    CHECK(view.GetStepCount() > 0);
    CHECK(view.GetComposeGroupCount() > 0);
    CHECK(view.GetProviderSlotCount() > 0);
    CHECK(!view.GetBoundPaths().empty());
    CHECK(view.GetPathTexts().size() == view.GetBoundPaths().size());
    CHECK(!view.GetOpGraph().empty());
    CHECK(!view.GetControlPaths().empty());
    CHECK(view.GetProgramDigest(evaluator.GetCompiledRigSignature()) != 0);
    CHECK(view.GetConstantDigest() != 0);
    CHECK(view.GetChainCount() > 0);
    CHECK(view.GetChainTargets().count(SdfPath("/View/Channels.dial")) > 0);
    CHECK(view.GetRevisionCount() > 0);
    for (size_t chain = 0; chain < view.GetChainCount(); ++chain) {
        CHECK(!view.GetChainFacts(chain).target.IsEmpty());
        for (size_t revision = 0; revision < view.GetChainRevisionCount(chain);
             ++revision) {
            CHECK(!view.GetChainRevisionFacts(chain, revision)
                       .moverPath.IsEmpty());
        }
    }
    CHECK(view.GetWeightObjectCount() == 1);
    if (view.GetWeightObjectCount() > 0) {
        CHECK(view.GetWeightObjectFacts(0).path ==
              SdfPath("/View/Rig/Weights/PanelEdge"));
    }
    CHECK(view.GetPropertyChainCount() > 0);
    CHECK(view.GetBoundInputCount() > 0);
    CHECK(view.GetAvarSlotCount() > 0);

    std::shared_ptr<const RigExecCompiledProgram> held =
        program->ShareCompiledProgram();
    CHECK(held != nullptr);
    if (!held) {
        return;
    }
    CHECK(held == program->ShareCompiledProgram());
    CHECK(held->IsSameVersionAs(view));
    CHECK(held->GetStepCount() == view.GetStepCount());

    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1.0));
    CHECK(pose.valid);
    CHECK(pose.diagnostics.empty());
    CHECK(pose.controlFrames.count(SdfPath("/View/Rig/Joint")) > 0);
    const auto dialed =
        pose.movedProperties.find(SdfPath("/View/Channels.dial"));
    const bool dialedFound = dialed != pose.movedProperties.end();
    CHECK(dialedFound);
    if (dialedFound) {
        const bool dialedIsDouble = dialed->second.IsHolding<double>();
        CHECK(dialedIsDouble);
        if (dialedIsDouble) {
            CHECK(dialed->second.Get<double>() == 6.0);
        }
    }

    // Nothing a consumer can reach is a writable execution record.
    CHECK(_WritesSolverScratch(view, 0) == 1);
    CHECK(_ReadsInterpolatorScratch(view, 0) == 1);
    CHECK(_LendsChainRecord(view, 0) == 1);
    CHECK(_LendsRevisionRecord(view, 0) == 1);
    CHECK(_LendsWeightRecord(view, 0) == 1);
    CHECK(_ReachesStepGraph(*program, 0) == 1);
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(_ReadsFrozenHandle(frozen, 0) == 1);
    CHECK(_NamesImplType<RigExecBakedProgramImpl>(0, 0) == 1);
    CHECK(view.GetStepCount() > 0);
}

} // namespace

int
main()
{
    TestSeal();
    if (failures > 0) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("installed sealed-view consumer ok\n");
    return 0;
}
