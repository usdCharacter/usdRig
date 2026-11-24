// The scene program gives the same bytes for every op batch size and worker
// thread count: every version of every chain, at every frame, matches the
// run with unlimited batches on the calling thread.
//   testRigExecSceneDeterminism <stage>...

#include "rigExec/api/taskSystem.h"
#include "rigExecScene/sceneDescriptors.h"
#include "rigExecScene/sceneGraphBinding.h"
#include "rigExecScene/sceneProgramTranslation.h"
#include "rigExecScene/usdSceneAccess.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/matrix4f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

int gFailures = 0;

template <class T>
bool
SameBytes(const VtValue &a, const VtValue &b, bool *known)
{
    if (!a.IsHolding<T>())
        return false;
    *known = true;
    if (!b.IsHolding<T>())
        return false;
    const T &x = a.UncheckedGet<T>();
    const T &y = b.UncheckedGet<T>();
    return std::memcmp(&x, &y, sizeof(T)) == 0;
}

template <class T>
bool
SameArrayBytes(const VtValue &a, const VtValue &b, bool *known)
{
    if (!a.IsHolding<VtArray<T>>())
        return false;
    *known = true;
    if (!b.IsHolding<VtArray<T>>())
        return false;
    const VtArray<T> &x = a.UncheckedGet<VtArray<T>>();
    const VtArray<T> &y = b.UncheckedGet<VtArray<T>>();
    return x.size() == y.size() &&
           (x.empty() ||
            std::memcmp(x.cdata(), y.cdata(), x.size() * sizeof(T)) == 0);
}

// Byte equality for the value types chains carry; operator== elsewhere.
bool
Identical(const VtValue &a, const VtValue &b)
{
    bool known = false;
    const bool same =
        SameArrayBytes<GfVec3f>(a, b, &known) ||
        SameArrayBytes<GfVec3d>(a, b, &known) ||
        SameArrayBytes<float>(a, b, &known) ||
        SameArrayBytes<double>(a, b, &known) ||
        SameArrayBytes<GfMatrix4d>(a, b, &known) ||
        SameArrayBytes<GfMatrix4f>(a, b, &known) ||
        SameBytes<GfMatrix4d>(a, b, &known) ||
        SameBytes<GfMatrix4f>(a, b, &known) ||
        SameBytes<GfVec3f>(a, b, &known) || SameBytes<GfVec3d>(a, b, &known) ||
        SameBytes<float>(a, b, &known) || SameBytes<double>(a, b, &known);
    return known ? same : a == b;
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse())
        if (prim.GetTypeName() == "RigExecRoot")
            return prim.GetPath();
    return SdfPath();
}

struct Config {
    size_t maxBatchNodes = 0;
    unsigned threads = 1;
};

// Every chain version's value, by frame, in chain order.
using Frames = std::vector<std::vector<VtValue>>;

bool
Run(const RigExecSceneProgram &program, size_t frameCount, const Config &config,
    Frames *out)
{
    std::unique_ptr<RigExecTaskSystem> tasks;
    RigExecSceneProgramRuntime runtime;
    runtime.opCompile.maxBatchNodes = config.maxBatchNodes;
    if (config.threads > 1) {
        tasks = RigExecCreateThreadTaskSystem(config.threads);
        runtime.opEvaluate.tasks = tasks.get();
    }
    std::string error;
    if (!runtime.Prepare(program, &error)) {
        std::fprintf(stderr, "  prepare failed: %s\n", error.c_str());
        return false;
    }
    for (size_t identity = 0; identity < frameCount; ++identity) {
        if (!runtime.Evaluate(program, identity, {}, &error)) {
            std::fprintf(stderr, "  frame %zu failed: %s\n", identity,
                         error.c_str());
            return false;
        }
        runtime.PushHistory();
        std::vector<VtValue> frame;
        for (const auto &entry : program.layout.routes.chains) {
            const auto &chain = entry.second;
            std::vector<RigExecValueId> ids = {chain.base, chain.poseBase};
            for (const auto &version : chain.versions)
                ids.push_back(version.value);
            for (const RigExecValueId id : ids) {
                VtValue value;
                if (id < runtime.values.values.size())
                    RigExecReadSceneGraphValue(runtime.values, id, &value);
                frame.push_back(std::move(value));
            }
        }
        out->push_back(std::move(frame));
    }
    return true;
}

void
Check(const std::string &stagePath)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    const SdfPath rig = stage ? FindRig(stage) : SdfPath();
    if (rig.IsEmpty()) {
        std::fprintf(stderr, "FAIL %s: no rig\n", stagePath.c_str());
        ++gFailures;
        return;
    }
    double start = 1.0, end = 12.0;
    if (stage->HasAuthoredTimeCodeRange() &&
        stage->GetEndTimeCode() > stage->GetStartTimeCode()) {
        start = stage->GetStartTimeCode();
        end = std::min(stage->GetEndTimeCode(), start + 23.0);
    }
    std::vector<UsdTimeCode> identities;
    for (double t = start; t <= end; t += 1.0)
        identities.push_back(UsdTimeCode(t));
    const size_t frameCount = identities.size();
    identities.push_back(UsdTimeCode::Default());

    const RigExecUsdSceneAccess access(stage);
    RigExecSceneDescriptors scene;
    RigExecSceneProgram program;
    std::string error;
    if (!RigExecCaptureSceneDescriptors(access, rig, identities, &scene,
                                        &error) ||
        !RigExecTranslateSceneProgram(scene, &program, &error)) {
        std::fprintf(stderr, "FAIL %s: scene program: %s\n", stagePath.c_str(),
                     error.c_str());
        ++gFailures;
        return;
    }

    Frames reference;
    if (!Run(program, frameCount, Config(), &reference)) {
        std::fprintf(stderr, "FAIL %s: reference run\n", stagePath.c_str());
        ++gFailures;
        return;
    }
    const unsigned hardware = std::max(1u, std::thread::hardware_concurrency());
    for (const size_t batch : {size_t(0), size_t(1), size_t(3)}) {
        for (const unsigned threads : {1u, 2u, 3u, hardware}) {
            if (threads > hardware || (batch == 0 && threads == 1))
                continue;
            Frames frames;
            if (!Run(program, frameCount, {batch, threads}, &frames)) {
                std::fprintf(stderr, "FAIL %s: batch %zu, %u threads\n",
                             stagePath.c_str(), batch, threads);
                ++gFailures;
                continue;
            }
            for (size_t f = 0; f < frameCount; ++f) {
                bool same = frames[f].size() == reference[f].size();
                for (size_t v = 0; same && v < frames[f].size(); ++v)
                    same = Identical(reference[f][v], frames[f][v]);
                if (!same) {
                    std::fprintf(stderr,
                                 "FAIL %s: frame %zu differs with batch %zu, "
                                 "%u threads\n",
                                 stagePath.c_str(), f, batch, threads);
                    ++gFailures;
                    break;
                }
            }
        }
    }
    std::printf("checked %s (%zu frames, %zu operations, %zu values)\n",
                TfGetBaseName(stagePath).c_str(), frameCount,
                program.operations.size(), reference.front().size());
}

} // namespace

int
main(int argc, char **argv)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    PlugRegistry::GetInstance().RegisterPlugins(
        TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR));
#endif
    for (int i = 1; i < argc; ++i)
        Check(argv[i]);
    if (gFailures) {
        std::fprintf(stderr, "testRigExecSceneDeterminism: %d failure(s)\n",
                     gFailures);
        return 1;
    }
    std::printf("testRigExecSceneDeterminism: ok\n");
    return 0;
}
