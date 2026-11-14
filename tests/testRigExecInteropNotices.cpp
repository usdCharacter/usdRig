// Compile notices for UsdSkel data RigExec does not reproduce: unnormalized
// skin weights, constant-interpolation skin bindings, and blend shape
// inbetweens / normalOffsets.
#include "rigExec/rigEvaluator.h"

#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdSkel/blendShape.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

static bool
Mentions(const std::vector<std::string> &messages, const std::string &needle)
{
    for (const std::string &m : messages)
        if (m.find(needle) != std::string::npos)
            return true;
    return false;
}

static UsdStageRefPtr
SkinStage(const VtIntArray &indices, const VtFloatArray &weights,
          int elementSize)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"))
        .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray,
                         false)
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)});
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    for (const char *name : {"A", "B"}) {
        stage->DefinePrim(SdfPath(std::string("/Asset/Rig/Joints/") + name),
                          TfToken("RigExecJoint"));
    }
    const UsdPrim mover = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Skin"),
                                            TfToken("RigExecSkinMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.CreateRelationship(TfToken("rigExec:moves"), false)
        .SetTargets({SdfPath("/Asset/Geom/P.points")});
    mover.CreateRelationship(TfToken("rigExec:influences"), false)
        .SetTargets(
            {SdfPath("/Asset/Rig/Joints/A"), SdfPath("/Asset/Rig/Joints/B")});
    mover
        .CreateAttribute(TfToken("rigExec:elementSize"), SdfValueTypeNames->Int,
                         false)
        .Set(elementSize);
    mover
        .CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray, false)
        .Set(indices);
    mover
        .CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray, false)
        .Set(weights);
    return stage;
}

static void
TestSkinNotices()
{
    {
        RigExecRigEvaluator evaluator(
            SkinStage({0, 1, 0, 1, 0, 1}, {1, 0, 0.5f, 0.5f, 0.25f, 0.25f}, 2),
            SdfPath("/Asset/Rig"));
        std::vector<std::string> notes;
        CHECK(evaluator.Compile(&notes));
        CHECK(Mentions(notes, "do not sum to 1"));
        CHECK(Mentions(notes, "1 point(s)"));
    }
    {
        RigExecRigEvaluator evaluator(
            SkinStage({0, 1, 0, 1, 0, 1}, {1, 0, 0.5f, 0.5f, 0.5f, 0.5f}, 2),
            SdfPath("/Asset/Rig"));
        std::vector<std::string> notes;
        CHECK(evaluator.Compile(&notes));
        CHECK(!Mentions(notes, "do not sum to 1"));
    }
    {
        // A rigid (constant) binding: one influence set for three points.
        RigExecRigEvaluator evaluator(SkinStage({0, 1}, {0.5f, 0.5f}, 2),
                                      SdfPath("/Asset/Rig"));
        std::vector<std::string> notes;
        CHECK(evaluator.Compile(&notes));
        CHECK(!evaluator.GetSkippedOperations().empty());
        CHECK(Mentions(notes, "constant interpolation"));
    }
}

static void
TestBlendShapeNotices()
{
    for (const bool authored : {false, true}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        stage->DefinePrim(SdfPath("/Asset/Geom/P"), TfToken("Points"))
            .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray,
                             false)
            .Set(VtVec3fArray{GfVec3f(0)});
        const UsdSkelBlendShape shape =
            UsdSkelBlendShape::Define(stage, SdfPath("/Asset/Shapes/S"));
        shape.CreateOffsetsAttr().Set(VtVec3fArray{GfVec3f(1, 0, 0)});
        if (authored) {
            shape.CreateNormalOffsetsAttr().Set(VtVec3fArray{GfVec3f(0, 1, 0)});
            shape.CreateInbetween(TfToken("half"));
        }
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        const UsdPrim input = stage->DefinePrim(
            SdfPath("/Asset/Rig/BlendInputs/B"), TfToken("RigExecBlendInput"));
        input
            .CreateAttribute(TfToken("inputs:weight"), SdfValueTypeNames->Float,
                             false)
            .Set(1.0f);
        const UsdPrim sample =
            stage->DefinePrim(SdfPath("/Asset/Rig/BlendInputs/B/S"),
                              TfToken("RigExecBlendSample"));
        sample
            .CreateAttribute(TfToken("rigExec:activation"),
                             SdfValueTypeNames->Float, false)
            .Set(1.0f);
        sample.CreateRelationship(TfToken("rigExec:blendShape"), false)
            .SetTargets({shape.GetPath()});
        input.CreateRelationship(TfToken("rigExec:samples"), false)
            .SetTargets({sample.GetPath()});
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/B"), TfToken("RigExecBlendShapeMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.CreateRelationship(TfToken("rigExec:moves"), false)
            .SetTargets({SdfPath("/Asset/Geom/P.points")});
        mover.CreateRelationship(TfToken("rigExec:blendInputs"), false)
            .SetTargets({input.GetPath()});

        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> notes;
        CHECK(evaluator.Compile(&notes));
        CHECK(Mentions(notes, "/Asset/Shapes/S: UsdSkelBlendShape authors") ==
              authored);
        CHECK(Mentions(notes, "inbetweens and normalOffsets") == authored);
    }
}

int
main()
{
    TestSkinNotices();
    TestBlendShapeNotices();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    return 0;
}
