// benchRigSpeed: compile and per-frame evaluation time for one rig.
//   benchRigSpeed <stage> [runs] [warmup] [frames] [rigPath]
// Compile runs build a fresh evaluator each time. "compile" is Compile();
// "ready" is Compile() plus the first Evaluate, which builds the program.
// "exec" is the mean Evaluate time per frame over the stage's frame range
// (or frames 1..[frames] when it has none), one sample per pass.
// Prints one RESULT line per phase: median, min, max and p10/p90 in ms.

#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

double
NowMs()
{
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

double
Quantile(std::vector<double> samples, double q)
{
    std::sort(samples.begin(), samples.end());
    const size_t index = std::min(
        samples.size() - 1, size_t(q * double(samples.size() - 1) + 0.5));
    return samples[index];
}

void
Report(const char *rig, const char *phase, const std::vector<double> &ms)
{
    std::printf("RESULT rig=%s phase=%s n=%zu median_ms=%.4f min_ms=%.4f "
                "max_ms=%.4f p10_ms=%.4f p90_ms=%.4f\n",
                rig, phase, ms.size(), Quantile(ms, 0.5), Quantile(ms, 0.0),
                Quantile(ms, 1.0), Quantile(ms, 0.1), Quantile(ms, 0.9));
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: benchRigSpeed <stage> [runs] [warmup] "
                             "[frames] [rigPath]\n");
        return 2;
    }
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    PlugRegistry::GetInstance().RegisterPlugins(
        TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR));
#endif
    const std::string stagePath = argv[1];
    const int runs = argc > 2 ? std::atoi(argv[2]) : 15;
    const int warmup = argc > 3 ? std::atoi(argv[3]) : 3;
    const int defaultFrames = argc > 4 ? std::atoi(argv[4]) : 48;

    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        std::fprintf(stderr, "cannot open %s\n", stagePath.c_str());
        return 1;
    }
    const SdfPath rigPath =
        argc > 5 ? SdfPath(argv[5]) : FindRig(stage);
    if (rigPath.IsEmpty()) {
        std::fprintf(stderr, "no RigExecRoot in %s\n", stagePath.c_str());
        return 1;
    }
    double start = 1.0;
    double end = double(defaultFrames);
    if (stage->HasAuthoredTimeCodeRange() &&
        stage->GetEndTimeCode() > stage->GetStartTimeCode()) {
        start = stage->GetStartTimeCode();
        end = stage->GetEndTimeCode();
    }
    std::vector<UsdTimeCode> frames;
    for (double t = start; t <= end; t += 1.0) {
        frames.emplace_back(t);
    }
    const std::string rigName = TfGetBaseName(stagePath);

    std::vector<double> compileMs;
    std::vector<double> readyMs;
    for (int i = 0; i < warmup + runs; ++i) {
        auto evaluator = std::make_unique<RigExecRigEvaluator>(stage, rigPath);
        std::vector<std::string> errors;
        const double t0 = NowMs();
        if (!evaluator->Compile(&errors)) {
            std::fprintf(stderr, "compile failed: %s\n",
                         errors.empty() ? "" : errors.front().c_str());
            return 1;
        }
        const double t1 = NowMs();
        const RigExecRigPose pose = evaluator->Evaluate(frames.front());
        const double t2 = NowMs();
        if (!pose.valid) {
            std::fprintf(stderr, "first evaluate invalid\n");
            for (const std::string &line : pose.diagnostics) {
                std::fprintf(stderr, "  %s\n", line.c_str());
            }
            return 1;
        }
        if (i >= warmup) {
            compileMs.push_back(t1 - t0);
            readyMs.push_back(t2 - t0);
        }
    }

    RigExecRigEvaluator evaluator(stage, rigPath);
    if (!evaluator.Compile()) {
        return 1;
    }
    std::vector<double> execMs;
    size_t invalid = 0;
    for (int i = 0; i < warmup + runs; ++i) {
        const double t0 = NowMs();
        for (const UsdTimeCode &time : frames) {
            invalid += evaluator.Evaluate(time).valid ? 0 : 1;
        }
        const double t1 = NowMs();
        if (i >= warmup) {
            execMs.push_back((t1 - t0) / double(frames.size()));
        }
    }
    if (invalid) {
        std::fprintf(stderr, "%zu invalid evaluations\n", invalid);
        return 1;
    }
    std::printf("RIG rig=%s path=%s frames=%zu\n", rigName.c_str(),
                rigPath.GetText(), frames.size());
    Report(rigName.c_str(), "compile", compileMs);
    Report(rigName.c_str(), "ready", readyMs);
    Report(rigName.c_str(), "exec", execMs);
    return 0;
}
