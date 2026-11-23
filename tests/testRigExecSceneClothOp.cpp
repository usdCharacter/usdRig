// The scene program runs the cloth mover on its op: sequential play that
// pushes each frame, and jumps that rebuild the history, give the native
// evaluator's points bit for bit, with and without constraint sets.
#include "rigExec/api/rigEvaluator.h"

#include "pxr/base/plug/registry.h"

#include <cstdio>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

static int failures = 0;
#define CHECK(x)                                                               \
    do {                                                                       \
        if (!(x)) {                                                            \
            std::printf("FAIL %d: %s\n", __LINE__, #x);                        \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

#include "rigExecClothStage.h"
#include "rigExecSceneProgramDrive.h"

namespace {

using namespace rigExecClothStage;

/// The curtain with a stretch set, a painted damping set and an attachment
/// between two bindings.
UsdStageRefPtr
ConstraintStage()
{
    auto stage = MakeStage();
    auto cloth = stage->GetPrimAtPath(kCurtain.GetPrimPath());
    auto mover = stage->GetPrimAtPath(kCloth);
    VtVec3fArray rest;
    cloth.GetAttribute(TfToken("points")).Get(&rest);
    VtVec2fArray uv;
    for (const auto &point : rest)
        uv.push_back(GfVec2f(point[0], point[1]));
    auto st = cloth.CreateAttribute(TfToken("primvars:st"),
                                    SdfValueTypeNames->TexCoord2fArray);
    st.Set(uv);
    st.SetMetadata(TfToken("interpolation"), VtValue(TfToken("vertex")));
    mover.CreateRelationship(TfToken("rigExec:uvSet"))
        .SetTargets({st.GetPath()});
    const auto profile = [&](const char *name, const char *kind) {
        auto prim = stage->DefinePrim(kCloth.AppendChild(TfToken(name)),
                                      TfToken("RigExecClothConstraintSet"));
        prim.CreateAttribute(TfToken("rigExec:kind"), SdfValueTypeNames->Token)
            .Set(TfToken(kind));
        return prim;
    };
    auto stretch = profile("Stretch", "stretch");
    stretch.CreateAttribute(TfToken("inputs:warp"), SdfValueTypeNames->Float)
        .Set(4.0f);
    stretch.CreateAttribute(TfToken("inputs:weft"), SdfValueTypeNames->Float)
        .Set(.5f);
    auto damping = profile("Damping", "damping");
    damping
        .CreateAttribute(TfToken("inputs:minStrength"),
                         SdfValueTypeNames->Float)
        .Set(8.0f);
    damping
        .CreateAttribute(TfToken("inputs:maxStrength"),
                         SdfValueTypeNames->Float)
        .Set(8.0f);
    VtFloatArray painted(rest.size());
    for (size_t i = 0; i < painted.size(); ++i)
        painted[i] = float(i % 3) * 0.5f;
    auto paint = cloth.CreateAttribute(TfToken("primvars:damping"),
                                       SdfValueTypeNames->FloatArray);
    paint.Set(painted);
    damping.CreateRelationship(TfToken("rigExec:maskMap"))
        .SetTargets({paint.GetPath()});
    auto a = stage->DefinePrim(kCloth.AppendChild(TfToken("A")),
                               TfToken("RigExecClothBinding"));
    a.CreateAttribute(TfToken("rigExec:indices"), SdfValueTypeNames->IntArray)
        .Set(VtIntArray{42});
    auto b = stage->DefinePrim(kCloth.AppendChild(TfToken("B")),
                               TfToken("RigExecClothBinding"));
    b.CreateAttribute(TfToken("rigExec:mode"), SdfValueTypeNames->Token)
        .Set(TfToken("uv"));
    b.CreateAttribute(TfToken("rigExec:coordinates"),
                      SdfValueTypeNames->Float3Array)
        .Set(VtVec3fArray{GfVec3f(0, 40, 0)});
    auto attachment = profile("Link", "attachment");
    attachment.CreateRelationship(TfToken("rigExec:sourceBinding"))
        .SetTargets({a.GetPath()});
    attachment.CreateRelationship(TfToken("rigExec:targetBinding"))
        .SetTargets({b.GetPath()});
    mover.CreateRelationship(TfToken("rigExec:constraints"))
        .SetTargets(
            {stretch.GetPath(), damping.GetPath(), attachment.GetPath()});
    return stage;
}

void
Compare(const UsdStageRefPtr &stage, const char *what)
{
    using rigExecSceneProgramDrive::SceneRun;
    RigExecRigEvaluator native(stage, kRig);
    std::vector<std::string> errors;
    CHECK(native.Compile(&errors));
    for (const std::string &e : errors)
        std::printf("compile: %s\n", e.c_str());
    std::vector<Points> want(25);
    for (int t = 1; t <= 24; ++t)
        want[size_t(t)] = PointsOf(native.Evaluate(UsdTimeCode(t)), kCurtain);
    const auto curtain = [](const SceneRun &scene) {
        const VtValue points =
            scene.Final(kCurtain, RigExecSceneValueDomain::Points);
        return points.IsHolding<VtVec3fArray>()
                   ? Points(points.UncheckedGet<VtVec3fArray>().begin(),
                            points.UncheckedGet<VtVec3fArray>().end())
                   : Points();
    };
    SceneRun scene(stage, kRig, -70, 24);
    CHECK(scene.ok);
    if (!scene.ok)
        return;
    for (int t = -70; t <= 24; ++t) {
        CHECK(scene.Evaluate(t));
        scene.runtime.PushHistory();
        if (t >= 1 && !SameBits(curtain(scene), want[size_t(t)])) {
            std::printf("%s, t=%d: the scene program differs by %g\n", what, t,
                        Furthest(curtain(scene), want[size_t(t)]));
            CHECK(false);
            break;
        }
    }
    SceneRun jumping(stage, kRig, -70, 24);
    for (int t : {17, 5, 24, 1}) {
        CHECK(jumping.Jump(t));
        if (!SameBits(curtain(jumping), want[size_t(t)])) {
            std::printf("%s, jump to %d: the scene program differs by %g\n",
                        what, t, Furthest(curtain(jumping), want[size_t(t)]));
            CHECK(false);
        }
    }
}

} // namespace

int
main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    Compare(MakeStage(), "curtain");
    Compare(ConstraintStage(), "constraint sets");
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
