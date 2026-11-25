// A property mover translated from USD and run as rigExecOps' math op
// gives the bits RigExecRunProperty gives, for every operation and target
// type, alone and chained, and both refuse the same records.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecScene/propertyOps.h"
#include "rigExecScene/propertySceneTranslation.h"
#include "rigExecScene/usdSceneAccess.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

template <class T>
bool
SameBits(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

const SdfPath kRig("/Rig");
const SdfPath kTarget("/Rig.target");

/// A rig with one target attribute of \p type and movers of \p moverType,
/// each authored by its callback, in stack order.
struct Scene {
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    std::vector<SdfPath> movers;
    RigExecSceneDescriptors descriptors;

    Scene(const SdfValueTypeName &type, const char *moverType,
          const std::vector<std::function<void(const UsdPrim &)>> &authors)
    {
        stage->DefinePrim(kRig, TfToken("RigExecRoot"))
            .CreateAttribute(TfToken("target"), type);
        for (size_t i = 0; i < authors.size(); ++i) {
            const SdfPath path("/Rig/M" + std::to_string(i));
            authors[i](stage->DefinePrim(path, TfToken(moverType)));
            movers.push_back(path);
        }
        std::string error;
        const RigExecUsdSceneAccess access(stage);
        CHECK(RigExecCaptureSceneDescriptors(
            access, kRig, {UsdTimeCode::Default()}, &descriptors, &error));
    }

    /// The mover's resolved descriptor; false where translation refuses.
    bool Resolve(size_t i, RigExecScenePropertyDescriptor *out, float *weight)
    {
        std::string error;
        bool enabled = false;
        return RigExecTranslateSceneProperty(descriptors, movers[i], kTarget,
                                             out, &error) &&
               RigExecResolveSceneProperty(descriptors, out,
                                           UsdTimeCode::Default(), {}, &enabled,
                                           weight, &error) &&
               enabled;
    }
};

/// The movers as a chain of op nodes, the first one's In host-written.
struct Chain {
    std::vector<RigExecScenePropertyDescriptor> descriptors;
    std::vector<RigExecPropertyOpNode> nodes;
    std::unique_ptr<RigExecEvaluator> evaluator;

    explicit Chain(Scene &scene)
    {
        RigExecRigDescBuilder builder;
        for (size_t i = 0; i < scene.movers.size(); ++i) {
            RigExecScenePropertyDescriptor d;
            float weight = 0;
            CHECK(scene.Resolve(i, &d, &weight));
            RigExecPropertyOpNode node;
            std::string error;
            if (!RigExecAddPropertyOp(d, &builder,
                                      nodes.empty() ? nullptr : &nodes.back(),
                                      &node, &error)) {
                std::printf("add: %s\n", error.c_str());
                CHECK(false);
                return;
            }
            descriptors.push_back(d);
            nodes.push_back(node);
        }
        std::string error;
        auto desc = builder.Build(&error);
        auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
        evaluator =
            program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
        if (!evaluator)
            std::printf("chain: %s\n", error.c_str());
        CHECK(evaluator);
    }

    template <class T> bool Run(const T &in, float envelope, T *out)
    {
        std::string error;
        if (!evaluator || !evaluator->SetInput(nodes.front().in, &in, &error))
            return false;
        for (size_t i = 0; i < nodes.size(); ++i)
            if (!RigExecWritePropertyOpInputs(descriptors[i], nodes[i],
                                              envelope, evaluator.get(),
                                              &error))
                return false;
        if (!evaluator->Evaluate(&error)) {
            std::printf("evaluate: %s\n", error.c_str());
            return false;
        }
        *out = *static_cast<const T *>(evaluator->GetOutput(nodes.back().out));
        return true;
    }
};

const char *const kFloatOps[] = {"add",   "multiply", "clamp", "remap",
                                 "blend", "quantize", "curve"};
const float kEnvelopes[] = {1.0f, 0.3f, 0.0f};

std::function<void(const UsdPrim &)>
FloatMover(const char *op, float value)
{
    return [op, value](const UsdPrim &m) {
        m.GetAttribute(TfToken("rigExec:operation")).Set(TfToken(op));
        m.GetAttribute(TfToken("inputs:value")).Set(value);
        m.GetAttribute(TfToken("inputs:min")).Set(-0.5f);
        m.GetAttribute(TfToken("inputs:max")).Set(1.75f);
        if (std::string(op) == "curve") {
            m.GetAttribute(TfToken("inputs:keys"))
                .Set(VtVec2fArray{GfVec2f(-1, 2), GfVec2f(0, 0),
                                  GfVec2f(0.5f, 1), GfVec2f(3, -4)});
            m.GetAttribute(TfToken("inputs:tangents"))
                .Set(VtVec2fArray{GfVec2f(1, -2), GfVec2f(0, 0.5f),
                                  GfVec2f(3, 1), GfVec2f(-1, 2)});
        }
    };
}

void
TestFloat()
{
    for (const char *op : kFloatOps) {
        Scene scene(SdfValueTypeNames->Float, "RigExecFloatMathMover",
                    {FloatMover(op, 1.25f)});
        Chain chain(scene);
        for (const float envelope : kEnvelopes)
            for (float base = -3.0f; base <= 4.0f; base += 0.375f) {
                float expected = 0, actual = 0;
                CHECK(RigExecRunProperty(std::get<RigExecFloatPropertyRecord>(
                                             chain.descriptors.front().record),
                                         base, envelope, &expected));
                CHECK(chain.Run(base, envelope, &actual));
                if (!SameBits(expected, actual)) {
                    std::printf("%s base %g env %g: %.9g vs %.9g\n", op, base,
                                envelope, expected, actual);
                    CHECK(false);
                }
            }
    }
}

void
TestVec3f()
{
    for (const char *op : {"add", "multiply", "clamp", "remap", "blend"}) {
        Scene scene(
            SdfValueTypeNames->Float3, "RigExecVec3fMathMover",
            {[op](const UsdPrim &m) {
                m.GetAttribute(TfToken("rigExec:operation")).Set(TfToken(op));
                m.GetAttribute(TfToken("inputs:value"))
                    .Set(GfVec3f(1.25f, -2.0f, 0.5f));
                m.GetAttribute(TfToken("inputs:min"))
                    .Set(GfVec3f(-1.0f, -1.0f, 0.0f));
                m.GetAttribute(TfToken("inputs:max"))
                    .Set(GfVec3f(2.0f, 0.5f, 3.0f));
            }});
        Chain chain(scene);
        for (const float envelope : kEnvelopes)
            for (float t = -2.0f; t <= 2.0f; t += 0.5f) {
                const GfVec3f base(t, 2.0f * t - 1.0f, 0.25f - t);
                GfVec3f expected;
                CHECK(RigExecRunProperty(std::get<RigExecVec3PropertyRecord>(
                                             chain.descriptors.front().record),
                                         base, envelope, &expected));
                const rigExecStd::Vec3f in(base[0], base[1], base[2]);
                rigExecStd::Vec3f out;
                CHECK(chain.Run(in, envelope, &out));
                CHECK(SameBits(expected, GfVec3f(out[0], out[1], out[2])));
            }
    }
}

void
TestMatrix()
{
    GfMatrix4d value(1.0);
    value.SetRotate(GfRotation(GfVec3d(0, 1, 0), 30.0));
    value.SetTranslateOnly(GfVec3d(1, 2, 3));
    for (const char *op : {"multiply", "blend"}) {
        Scene scene(
            SdfValueTypeNames->Matrix4d, "RigExecMatrixMathMover",
            {[op, value](const UsdPrim &m) {
                m.GetAttribute(TfToken("rigExec:operation")).Set(TfToken(op));
                m.GetAttribute(TfToken("inputs:value")).Set(value);
            }});
        Chain chain(scene);
        GfMatrix4d base(1.0);
        base.SetRotate(GfRotation(GfVec3d(1, 0, 0), -45.0));
        base.SetTranslateOnly(GfVec3d(-2, 0.5, 4));
        for (const float envelope : kEnvelopes) {
            GfMatrix4d expected;
            CHECK(RigExecRunProperty(std::get<RigExecMatrixPropertyRecord>(
                                         chain.descriptors.front().record),
                                     base, envelope, &expected));
            rigExecStd::Mat4d in, out;
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    in._mtx[r][c] = base[r][c];
            CHECK(chain.Run(in, envelope, &out));
            GfMatrix4d actual;
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    actual[r][c] = out._mtx[r][c];
            CHECK(SameBits(expected, actual));
        }
    }
}

/// Three movers on one target, wired op to op, are the three runs in turn.
void
TestChain()
{
    Scene scene(SdfValueTypeNames->Float, "RigExecFloatMathMover",
                {FloatMover("add", 0.75f), FloatMover("curve", 0.0f),
                 FloatMover("clamp", 0.0f)});
    Chain chain(scene);
    CHECK(chain.nodes.size() == 3);
    for (const float envelope : kEnvelopes)
        for (float base = -3.0f; base <= 4.0f; base += 0.5f) {
            float expected = base, actual = 0;
            for (const auto &d : chain.descriptors)
                CHECK(RigExecRunProperty(
                    std::get<RigExecFloatPropertyRecord>(d.record), expected,
                    envelope, &expected));
            CHECK(chain.Run(base, envelope, &actual));
            CHECK(SameBits(expected, actual));
        }
}

/// A record RigExecRunProperty passes through is refused, not run.
void
TestRefusals()
{
    Scene unknown(SdfValueTypeNames->Float, "RigExecFloatMathMover",
                  {FloatMover("nonsense", 1.0f)});
    RigExecScenePropertyDescriptor d;
    float weight = 0;
    CHECK(unknown.Resolve(0, &d, &weight));
    float out = 0;
    CHECK(!RigExecRunProperty(std::get<RigExecFloatPropertyRecord>(d.record),
                              1.0f, 1.0f, &out));
    RigExecRigDescBuilder builder;
    RigExecPropertyOpNode node;
    std::string error;
    CHECK(!RigExecAddPropertyOp(d, &builder, nullptr, &node, &error));

    Scene unordered(
        SdfValueTypeNames->Float, "RigExecFloatMathMover",
        {[](const UsdPrim &m) {
            m.GetAttribute(TfToken("rigExec:operation")).Set(TfToken("curve"));
            m.GetAttribute(TfToken("inputs:keys"))
                .Set(VtVec2fArray{GfVec2f(1, 0), GfVec2f(0, 1)});
        }});
    CHECK(unordered.Resolve(0, &d, &weight));
    CHECK(!RigExecRunProperty(std::get<RigExecFloatPropertyRecord>(d.record),
                              1.0f, 1.0f, &out));
    // Keys are per-frame inputs: the node builds, and writing them fails.
    RigExecRigDescBuilder curveBuilder;
    CHECK(RigExecAddPropertyOp(d, &curveBuilder, nullptr, &node, &error));
    auto desc = curveBuilder.Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(evaluator);
    CHECK(evaluator && !RigExecWritePropertyOpInputs(d, node, 1.0f,
                                                     evaluator.get(), &error));
    CHECK(error.find("curve keys") != std::string::npos);
}

} // namespace

int
main()
{
    CHECK(!PlugRegistry::GetInstance()
               .RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR)
               .empty());
    std::string registryError;
    if (!RigExecEnsureBuiltinRegistry(&registryError)) {
        std::printf("%s\n", registryError.c_str());
        return 1;
    }
    TestFloat();
    TestVec3f();
    TestMatrix();
    TestChain();
    TestRefusals();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
