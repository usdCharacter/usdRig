// Evaluation is bit-identical across thread counts: every moved property of
// every frame matches the single-threaded run byte for byte.
//   testRigExecDeterminism <stage>...

#include "rigExec/api/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/matrix4f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/work/threadLimits.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
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

// Byte equality for the value types a pose carries; operator== elsewhere.
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
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot")
            return prim.GetPath();
    }
    return SdfPath();
}

using Frames = std::vector<std::map<SdfPath, VtValue>>;

bool
Run(const UsdStageRefPtr &stage, const SdfPath &rig,
    const std::vector<double> &times, Frames *out)
{
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        std::fprintf(stderr, "  compile failed: %s\n",
                     errors.empty() ? "" : errors.front().c_str());
        return false;
    }
    for (const double t : times) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
        if (!pose.valid) {
            std::fprintf(stderr, "  frame %g invalid\n", t);
            return false;
        }
        out->push_back(pose.movedProperties);
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
    std::vector<double> times;
    double start = 1.0, end = 12.0;
    if (stage->HasAuthoredTimeCodeRange() &&
        stage->GetEndTimeCode() > stage->GetStartTimeCode()) {
        start = stage->GetStartTimeCode();
        end = std::min(stage->GetEndTimeCode(), start + 23.0);
    }
    for (double t = start; t <= end; t += 1.0)
        times.push_back(t);

    const unsigned hardware = WorkGetPhysicalConcurrencyLimit();
    const unsigned restore = WorkGetConcurrencyLimit();
    Frames reference;
    WorkSetConcurrencyLimit(1);
    const bool ran = Run(stage, rig, times, &reference);
    if (!ran) {
        WorkSetConcurrencyLimit(restore);
        std::fprintf(stderr, "FAIL %s: single-threaded run\n",
                     stagePath.c_str());
        ++gFailures;
        return;
    }
    for (const unsigned threads : {2u, 3u, hardware}) {
        if (threads < 2 || threads > hardware)
            continue;
        WorkSetConcurrencyLimit(threads);
        Frames frames;
        if (!Run(stage, rig, times, &frames)) {
            std::fprintf(stderr, "FAIL %s: %u threads\n", stagePath.c_str(),
                         threads);
            ++gFailures;
            continue;
        }
        for (size_t f = 0; f < times.size(); ++f) {
            const auto &a = reference[f];
            const auto &b = frames[f];
            bool same = a.size() == b.size();
            for (auto i = a.begin(), j = b.begin(); same && i != a.end();
                 ++i, ++j) {
                if (i->first != j->first || !Identical(i->second, j->second)) {
                    std::fprintf(stderr,
                                 "FAIL %s: %s differs at frame %g with %u "
                                 "threads\n",
                                 stagePath.c_str(), i->first.GetText(),
                                 times[f], threads);
                    same = false;
                }
            }
            if (!same) {
                ++gFailures;
                break;
            }
        }
    }
    WorkSetConcurrencyLimit(restore);
    std::printf("checked %s (%zu frames)\n", TfGetBaseName(stagePath).c_str(),
                times.size());
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
        std::fprintf(stderr, "testRigExecDeterminism: %d failure(s)\n",
                     gFailures);
        return 1;
    }
    std::printf("testRigExecDeterminism: ok\n");
    return 0;
}
