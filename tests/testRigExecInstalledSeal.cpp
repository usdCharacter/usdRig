// A consumer built against the installed SDK alone. The install ships
// rigExec/api/ and the internal headers listed in
// tools/rigexec_installed_internal.txt, nothing else under rigExec/. A host
// reads a compiled rig through RigExecProgramView; the evaluator's execution
// records are not nameable. Each probe is a pair of overloads; the first is
// viable only when the name is reachable through the installed headers, so a
// probe returning 1 is a record the seal keeps out.
#include "rigExec/api/bakedProgram.h"
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/frozenContext.h"
#include "rigExec/api/programView.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExecSchema/schemaTokens.h"

// Built only against the installed SDK, so these must not be reachable.
#if __has_include("rigExec/internal/compiledProgram.h")
#error "internal/compiledProgram.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/internal/bakedProgramImpl.h")
#error "internal/bakedProgramImpl.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/internal/frozenProgram.h")
#error "internal/frozenProgram.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/internal/frozenContextInternal.h")
#error "internal/frozenContextInternal.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/internal/bakedSchedule.h")
#error "internal/bakedSchedule.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/internal/bakedOpGraph.h")
#error "internal/bakedOpGraph.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/internal/compile/rigDescData.h")
#error "internal/compile/rigDescData.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/internal/program/program.h")
#error "internal/program/program.h is shipped to installed consumers"
#endif
#if __has_include("rigExec/api/backgroundScheduler.h")
#error "the background scheduler is shipped in the engine, not rigExecPlayback"
#endif
#if __has_include("rigExec/internal/program/alignedBuffer.h")
#error "internal/program/alignedBuffer.h is shipped to installed consumers"
#endif

#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/points.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
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
    static_assert(std::is_same<decltype(RigExecProgramView().Program()),
                               const RigExecBakedProgram *>::value,
                  "the program view lends a read-only program");
    const RigExecProgramView view = evaluator.GetProgramView();
    CHECK(static_cast<bool>(view));
    CHECK(view.Program() == program);
    CHECK(view.ClusterCount() > 0);
    CHECK(view.ClusterCount() == program->GetClusterCount());
    CHECK(view.LastTime() == UsdTimeCode::Default());
    const RigExecProgramView empty;
    CHECK(!empty);
    CHECK(empty.ClusterCount() == 0);
    CHECK(empty.LastTime() == UsdTimeCode::Default());

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
    CHECK(view.LastTime() == UsdTimeCode(1.0));
    CHECK(evaluator.GetProgramView().ClusterCount() == view.ClusterCount());

    // Nothing a consumer can reach is a writable execution record.
    CHECK(_ReachesStepGraph(*program, 0) == 1);
    CHECK(_ReachesStepGraph(*view.Program(), 0) == 1);
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    CHECK(_ReadsFrozenHandle(frozen, 0) == 1);
    CHECK(_NamesImplType<RigExecBakedProgramImpl>(0, 0) == 1);
    CHECK(_NamesImplType<RigExecCompiledProgram>(0, 0) == 1);
    CHECK(_NamesImplType<RigExecProgram>(0, 0) == 1);
    CHECK(_NamesImplType<RigExecEvaluatorState>(0, 0) == 1);
}

std::set<std::string>
ReadInstalledInternalList(const std::filesystem::path &listPath)
{
    std::set<std::string> listed;
    std::ifstream in(listPath);
    CHECK(in.good());
    std::string line;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
            line.pop_back();
        }
        if (!line.empty() && line.front() != '#') {
            listed.insert("internal/" + line);
        }
    }
    return listed;
}

// Under include/rigExec/ the install holds api/ and exactly the listed
// internal headers.
void
TestInstalledHeaderSet(const std::filesystem::path &includeRoot,
                       const std::filesystem::path &listPath)
{
    const std::filesystem::path rigExecDir = includeRoot / "rigExec";
    CHECK(std::filesystem::is_directory(rigExecDir / "api"));
    const std::set<std::string> listed = ReadInstalledInternalList(listPath);
    CHECK(!listed.empty());
    std::set<std::string> installedInternal;
    size_t apiHeaders = 0;
    for (const auto &entry :
         std::filesystem::recursive_directory_iterator(rigExecDir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string rel =
            entry.path().lexically_relative(rigExecDir).generic_string();
        if (rel.rfind("api/", 0) == 0 &&
            rel.find('/', 4) == std::string::npos) {
            ++apiHeaders;
        } else if (rel.rfind("internal/", 0) == 0) {
            installedInternal.insert(rel);
        } else {
            ++failures;
            std::printf("FAIL installed outside api/ and the internal list: "
                        "rigExec/%s\n",
                        rel.c_str());
        }
    }
    CHECK(apiHeaders > 0);
    for (const std::string &rel : installedInternal) {
        if (!listed.count(rel)) {
            ++failures;
            std::printf("FAIL unlisted internal header installed: rigExec/%s\n",
                        rel.c_str());
        }
    }
    for (const std::string &rel : listed) {
        if (!installedInternal.count(rel)) {
            ++failures;
            std::printf("FAIL listed internal header missing: rigExec/%s\n",
                        rel.c_str());
        }
    }
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc != 3) {
        std::printf("usage: %s <installed include dir> <internal list>\n",
                    argv[0]);
        return 2;
    }
    TestSeal();
    TestInstalledHeaderSet(argv[1], argv[2]);
    if (failures > 0) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("installed sealed-view consumer ok\n");
    return 0;
}
