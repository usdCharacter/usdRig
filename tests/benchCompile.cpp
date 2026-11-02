// benchCompile -- cold Compile() cost per example rig, plus Evaluate, bake,
// and shared scene-descriptor compile for the same rig.
//
// The dynamic-weight compile harness: dynamic-weight rigs pay the shared
// dependency-compiler / operation-graph builder cost on every load and
// recompile, so the headline number is cold Compile() wall time per rig.
// Reports the Compile stage; Evaluate is the validity check beside it.
// Prints human-readable numbers; asserts nothing, so it is built but
// deliberately NOT registered with ctest (like benchPlayback).
//
//   benchCompile [examplesDir] [reps] [--rigs a,b,c] [--profile RIG]
//     [--skip-bake] [--skip-standalone]
//
// examplesDir defaults to "examples" (run from the repo root). reps defaults
// to 5. --rigs restricts the run to a comma-separated subset, naming either
// a tour rig (02_TwoBoneIkLeg.usda), a dynweight extra (ArmRig.usda,
// ArmShotAnim.usda, dynamic_weight.usda), or any stage path. --profile
// compiles one rig once with profiling enabled and prints the compile/baked
// phase summary instead of timings (attribution runs separately from timing
// runs, so no profiler lock or overhead sits on the timed path).
//
// The default set is the 23-rig tour -- the numbered 01..19 plus the four
// top-level constraint stages (aimtest, aimtest_points, rotateConstraint,
// rigexec_flat) -- with three dynamic-weight extras on top:
// docs/examples/dynamic_weight.usda (resolved beside examplesDir),
// ArmRig.usda, and ArmShotAnim.usda. 02_TwoBoneIkLeg.usda is in both the
// tour and the dynamic-weight set. No biped rig carries a dynamic-weight
// node, so the tour's geomean is the no-dynweight guard metric.
//
// Columns, medians over reps unless noted:
//   compile_ms    cold Compile() on a fresh stage+evaluator (headline)
//   eval_ms       one Evaluate() at the probe time on the compiled rig
//   bake_ms       RigExecBakeToBinary wall (includes its internal compile;
//                 "refused" names an IsBakeable refusal, "FAIL" a bake error)
//   scenedesc_ms  RigExecCaptureSceneDescriptors over a pre-captured Db
//                 (the shared descriptor compiler; Db capture untimed)
// The summary prints the dynamic-weight subset and the geomean across the
// 23 tour rigs. Failures print their diagnostics and sit out the geomean.
//
// Environment: run from a shell with the repo env (bin/_env.sh) so PXR files
// and the schema resources resolve. Pin with taskset -c 5-9 for comparable
// medians; the bench itself is single-threaded apart from compile's own
// workers.
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecScene/sceneDescriptors.h"
#include "rigExecStandalone/sceneAccess.h"
#include "rigExecStandalone/sceneDb.h"

#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/resolveInfo.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/metrics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

using Clock = std::chrono::steady_clock;

double
Median(std::vector<double> samples)
{
    if (samples.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

double
Geomean(const std::vector<double> &samples)
{
    if (samples.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double total = 0.0;
    for (double v : samples) {
        total += std::log(v);
    }
    return std::exp(total / double(samples.size()));
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

UsdTimeCode
ProbeTime(const UsdStageRefPtr &stage)
{
    double start = 0.0;
    if (stage->HasAuthoredTimeCodeRange() ||
        stage->GetStartTimeCode() != stage->GetEndTimeCode()) {
        start = stage->GetStartTimeCode();
    }
    if (start == 0.0) {
        return UsdTimeCode::Default();
    }
    return UsdTimeCode(start);
}

VtDictionary
Metadata(const UsdObject &object)
{
    VtDictionary result;
    for (const auto &[key, value] : object.GetAllMetadata()) {
        result[key.GetString()] = value;
    }
    return result;
}

// In-memory Db capture, mirroring the standalone test's producer: the Db
// rows are the contract the descriptor compiler reads, so capturing once
// and timing RigExecCaptureSceneDescriptors alone keeps USD stage reads
// out of the measured standalone-compile number.
RigExecSceneDb
CaptureDb(const UsdStageRefPtr &stage, const std::vector<UsdTimeCode> &times)
{
    RigExecSceneDb db;
    db.upAxis = UsdGeomGetStageUpAxis(stage);
    for (const auto time : times) {
        db.identities.insert(RigExecStandaloneTimeKey(time));
    }
    db.prims[SdfPath::AbsoluteRootPath()] = {};
    for (const auto &child : stage->GetPseudoRoot().GetAllChildren()) {
        db.prims[SdfPath::AbsoluteRootPath()].children.push_back(
            child.GetPath());
    }
    for (const UsdPrim &prim : stage->Traverse()) {
        db.prims[prim.GetPath()] = {prim.GetTypeName(),
                                    prim.GetAppliedSchemas(), Metadata(prim),
                                    true};
        for (const auto &child : prim.GetAllChildren()) {
            db.prims[prim.GetPath()].children.push_back(child.GetPath());
        }
        for (const auto &attribute : prim.GetAttributes()) {
            RigExecStandaloneAttribute row;
            row.type = attribute.GetTypeName();
            row.metadata = Metadata(attribute);
            if (attribute.HasSpline()) {
                row.spline = VtValue(attribute.GetSpline());
            }
            attribute.GetConnections(&row.connections);
            row.hasValue = attribute.HasValue();
            row.variability = attribute.GetVariability();
            row.hasAuthoredValue = attribute.HasAuthoredValueOpinion();
            row.hasAuthoredReadableValue = attribute.HasAuthoredValue();
            row.hasAuthoredConnections = attribute.HasAuthoredConnections();
            row.mightBeTimeVarying = attribute.ValueMightBeTimeVarying();
            attribute.GetTimeSamples(&row.sampleTimes);
            for (const auto &spec :
                 attribute.GetPropertyStack(UsdTimeCode::Default())) {
                if (!spec->HasInfo(TfToken("default"))) {
                    continue;
                }
                row.hasAuthoredDefault = true;
                row.authoredDefault = spec->GetInfo(TfToken("default"));
                row.defaultBlocked =
                    row.authoredDefault.IsHolding<SdfValueBlock>();
                break;
            }
            for (const auto time : times) {
                VtValue value;
                attribute.Get(&value, time);
                if (attribute.GetResolveInfo(time).ValueIsBlocked()) {
                    row.blockedIdentities.insert(
                        RigExecStandaloneTimeKey(time));
                }
                row.resolved[RigExecStandaloneTimeKey(time)] = value;
            }
            db.attributes[attribute.GetPath()] = row;
        }
        for (const auto &relationship : prim.GetRelationships()) {
            RigExecStandaloneRelationship row;
            row.metadata = Metadata(relationship);
            relationship.GetTargets(&row.targets);
            db.relationships[relationship.GetPath()] = row;
        }
    }
    return db;
}

struct RigResult {
    std::string name;
    bool tour = false;
    bool dynweight = false;
    bool compiled = false;
    bool evalValid = false;
    double compileMs = 0.0;
    double compileMinMs = 0.0;
    double evalMs = 0.0;
    bool baked = false;
    bool bakeRefused = false;
    double bakeMs = 0.0;
    bool sceneDescOk = false;
    double sceneDescMs = 0.0;
    std::vector<std::string> diagnostics;
};

RigResult
TimeRig(const std::string &name, const std::string &path, int reps,
        bool wantBake, bool wantSceneDesc)
{
    RigResult result;
    result.name = name;
    // One untimed warmup so schema/plugin load lands outside the medians;
    // every timed rep still opens a fresh stage and a fresh evaluator.
    {
        const UsdStageRefPtr stage = UsdStage::Open(path);
        if (stage) {
            const SdfPath rig = FindRig(stage);
            if (!rig.IsEmpty()) {
                RigExecRigEvaluator warm(stage, rig);
                warm.Compile(nullptr);
            }
        }
    }
    std::vector<double> compiles;
    std::vector<double> evals;
    UsdTimeCode probe = UsdTimeCode::Default();
    for (int rep = 0; rep < reps; ++rep) {
        const UsdStageRefPtr stage = UsdStage::Open(path);
        if (!stage) {
            result.diagnostics.push_back("cannot open " + path);
            return result;
        }
        const SdfPath rig = FindRig(stage);
        if (rig.IsEmpty()) {
            result.diagnostics.push_back("no RigExecRoot in " + path);
            return result;
        }
        probe = ProbeTime(stage);
        RigExecRigEvaluator evaluator(stage, rig);
        std::vector<std::string> errors;
        const auto began = Clock::now();
        const bool ok = evaluator.Compile(&errors);
        const auto ended = Clock::now();
        if (!ok) {
            result.diagnostics = errors;
            return result;
        }
        compiles.push_back(
            std::chrono::duration<double>(ended - began).count() * 1000.0);
        const auto evalBegan = Clock::now();
        const RigExecRigPose pose = evaluator.Evaluate(probe);
        const auto evalEnded = Clock::now();
        evals.push_back(
            std::chrono::duration<double>(evalEnded - evalBegan).count() *
            1000.0);
        result.evalValid = pose.valid;
        if (!pose.valid && result.diagnostics.empty()) {
            result.diagnostics = pose.diagnostics;
        }
    }
    result.compiled = true;
    result.compileMs = Median(compiles);
    result.compileMinMs =
        *std::min_element(compiles.begin(), compiles.end());
    result.evalMs = Median(evals);

    if (wantBake) {
        // Bake on its own fresh evaluator: RigExecBakeToBinary compiles
        // internally, so this wall time covers the baked path end to end.
        std::vector<double> bakes;
        for (int rep = 0; rep < 2; ++rep) {
            const UsdStageRefPtr stage = UsdStage::Open(path);
            RigExecRigEvaluator evaluator(stage, FindRig(stage));
            RigExecBakeOpts opts;
            RigExecBakeResult baked;
            std::string error;
            const auto began = Clock::now();
            const bool ok =
                RigExecBakeToBinary(evaluator, opts, &baked, &error);
            const auto ended = Clock::now();
            if (!ok) {
                std::vector<std::string> reasons;
                evaluator.IsBakeable(&reasons);
                result.bakeRefused = !reasons.empty();
                if (!result.bakeRefused && result.diagnostics.empty()) {
                    result.diagnostics.push_back("bake: " + error);
                }
                break;
            }
            bakes.push_back(
                std::chrono::duration<double>(ended - began).count() *
                1000.0);
        }
        if (!bakes.empty()) {
            result.baked = true;
            result.bakeMs = Median(bakes);
        }
    }

    if (wantSceneDesc) {
        const UsdStageRefPtr stage = UsdStage::Open(path);
        if (stage) {
            const SdfPath rig = FindRig(stage);
            // Solver jointElements resolve at Default, so the Db carries
            // it alongside the probe time, like Build's own capture.
            std::vector<UsdTimeCode> descIdentities = {probe};
            if (probe != UsdTimeCode::Default()) {
                descIdentities.push_back(UsdTimeCode::Default());
            }
            const RigExecSceneDb db = CaptureDb(stage, descIdentities);
            const RigExecSceneDbAccess access(db);
            std::vector<double> descs;
            for (int rep = 0; rep < reps; ++rep) {
                RigExecSceneDescriptors scene;
                std::string error;
                const auto began = Clock::now();
                const bool ok = RigExecCaptureSceneDescriptors(
                    access, rig, descIdentities, &scene, &error);
                const auto ended = Clock::now();
                if (!ok) {
                    if (result.diagnostics.empty()) {
                        result.diagnostics.push_back("scenedesc: " + error);
                    }
                    break;
                }
                descs.push_back(
                    std::chrono::duration<double>(ended - began).count() *
                    1000.0);
            }
            if (!descs.empty()) {
                result.sceneDescOk = true;
                result.sceneDescMs = Median(descs);
            }
        }
    }
    return result;
}

void
ProfileRig(const std::string &path)
{
    const UsdStageRefPtr stage = UsdStage::Open(path);
    if (!stage) {
        std::printf("cannot open %s\n", path.c_str());
        return;
    }
    const SdfPath rig = FindRig(stage);
    if (rig.IsEmpty()) {
        std::printf("no RigExecRoot in %s\n", path.c_str());
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    evaluator.SetProfilingEnabled(true);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        std::printf("compile failed:\n");
        for (const auto &e : errors) {
            std::printf("  %s\n", e.c_str());
        }
        return;
    }
    evaluator.Evaluate(ProbeTime(stage));
    const auto rows = evaluator.GetProfiler().Summarize();
    std::printf("%-10s %-52s %6s %12s %12s\n", "category", "phase", "count",
                "total_ms", "max_ms");
    for (const auto &row : rows) {
        if (row.category != "compile" && row.category != "baked") {
            continue;
        }
        std::printf("%-10s %-52s %6zu %12.3f %12.3f\n",
                    row.category.c_str(), row.name.c_str(), row.count,
                    double(row.totalUs) / 1000.0,
                    double(row.maxUs) / 1000.0);
    }
}

}  // namespace

int
main(int argc, char **argv)
{
    std::string examplesDir = "examples";
    int reps = 5;
    bool wantBake = true;
    bool wantSceneDesc = true;
    std::string profileRig;
    std::vector<std::string> only;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--skip-bake") {
            wantBake = false;
        } else if (arg == "--skip-standalone") {
            wantSceneDesc = false;
        } else if (arg == "--profile" && i + 1 < argc) {
            profileRig = argv[++i];
        } else if (arg == "--rigs" && i + 1 < argc) {
            std::string list = argv[++i];
            size_t at = 0;
            while (at <= list.size()) {
                const size_t comma = list.find(',', at);
                only.push_back(list.substr(
                    at, comma == std::string::npos ? comma : comma - at));
                if (comma == std::string::npos) {
                    break;
                }
                at = comma + 1;
            }
        } else if (arg[0] != '-' && examplesDir == "examples" && i == 1) {
            examplesDir = arg;
        } else if (arg[0] != '-') {
            reps = std::atoi(arg.c_str());
        } else {
            std::printf("unknown flag %s\n", arg.c_str());
            return 2;
        }
    }
    // The 23-rig tour: the numbered 01..19 plus the four top-level
    // constraint stages. Dynamic-weight extras resolve beside it.
    const std::vector<std::string> tour = {
        "01_FkChainTail.usda",        "02_TwoBoneIkLeg.usda",
        "03_IkFkBlendClamp.usda",     "04_BlendShapeFace.usda",
        "05_TwistRibbonSpine.usda",   "06_LatticeBulge.usda",
        "07_SurfaceDrape.usda",       "08_AimEyes.usda",
        "09_PropertyMathMovers.usda", "10_AimXformTurret.usda",
        "11_VolumeWeights.usda",      "12_PatchCurve.usda",
        "13_ReadPhases.usda",         "14_VolumeConstrainedSweep.usda",
        "15_TransformMatrixMover.usda", "16_ConnectionReadPhases.usda",
        "17_PatchCurveShapes.usda",   "18_PatchMouth.usda",
        "19_HarmonicsChain.usda",     "aimtest.usda",
        "aimtest_points.usda",        "rotateConstraint.usda",
        "rigexec_flat.usda",
    };
    struct Extra {
        std::string name;
        std::string path;
    };
    const std::vector<Extra> extras = {
        {"dynamic_weight.usda", examplesDir + "/../docs/examples/dynamic_weight.usda"},
        {"ArmRig.usda", examplesDir + "/ArmRig.usda"},
        {"ArmShotAnim.usda", examplesDir + "/ArmShotAnim.usda"},
    };
    const auto isDyn = [](const std::string &name) {
        return name == "02_TwoBoneIkLeg.usda" || name == "ArmRig.usda" ||
               name == "ArmShotAnim.usda" || name == "dynamic_weight.usda";
    };
    const auto wanted = [&](const std::string &name, const std::string &path) {
        if (!profileRig.empty()) {
            return false;
        }
        if (only.empty()) {
            return true;
        }
        return std::find(only.begin(), only.end(), name) != only.end() ||
               std::find(only.begin(), only.end(), path) != only.end();
    };

    if (!profileRig.empty()) {
        std::string path = profileRig;
        if (path.find('/') == std::string::npos) {
            path = examplesDir + "/" + path;
            if (profileRig == "dynamic_weight.usda") {
                path = examplesDir + "/../docs/examples/dynamic_weight.usda";
            }
        }
        ProfileRig(path);
        return 0;
    }

    std::vector<RigResult> results;
    for (const auto &name : tour) {
        const std::string path = examplesDir + "/" + name;
        if (!wanted(name, path)) {
            continue;
        }
        RigResult r = TimeRig(name, path, reps, wantBake, wantSceneDesc);
        r.tour = true;
        r.dynweight = isDyn(name);
        results.push_back(std::move(r));
    }
    for (const auto &extra : extras) {
        if (!wanted(extra.name, extra.path)) {
            continue;
        }
        RigResult r =
            TimeRig(extra.name, extra.path, reps, wantBake, wantSceneDesc);
        r.tour = false;
        r.dynweight = true;
        results.push_back(std::move(r));
    }

    std::printf("%-28s %11s %11s %11s %11s %11s %s\n", "rig", "compile_ms",
                "compile_min", "eval_ms", "bake_ms", "scenedesc_ms",
                "status");
    std::vector<double> tourCompiles;
    std::vector<double> dynCompiles;
    for (const auto &r : results) {
        std::string bake = "n/a";
        if (r.baked) {
            bake = TfStringPrintf("%.3f", r.bakeMs);
        } else if (r.bakeRefused) {
            bake = "refused";
        } else if (wantBake && r.compiled) {
            bake = "FAIL";
        }
        std::string desc = "n/a";
        if (r.sceneDescOk) {
            desc = TfStringPrintf("%.3f", r.sceneDescMs);
        } else if (wantSceneDesc && r.compiled) {
            desc = "FAIL";
        }
        std::string status;
        if (!r.compiled) {
            status = "COMPILE-FAIL";
        } else if (!r.evalValid) {
            status = "EVAL-INVALID";
        } else {
            status = "ok";
        }
        if (r.dynweight) {
            status += " dynweight";
        }
        std::printf("%-28s %11.3f %11.3f %11.3f %11s %11s %s\n",
                    r.name.c_str(), r.compileMs, r.compileMinMs, r.evalMs,
                    bake.c_str(), desc.c_str(), status.c_str());
        if (r.compiled && r.tour) {
            tourCompiles.push_back(r.compileMs);
        }
        if (r.compiled && r.dynweight) {
            dynCompiles.push_back(r.compileMs);
        }
    }
    std::printf("tour_compile_geomean_ms %.4f (n=%zu)\n",
                Geomean(tourCompiles), tourCompiles.size());
    std::printf("dynweight_compile_geomean_ms %.4f (n=%zu)\n",
                Geomean(dynCompiles), dynCompiles.size());
    for (const auto &r : results) {
        for (const auto &d : r.diagnostics) {
            std::printf("diagnostic %s: %s\n", r.name.c_str(), d.c_str());
        }
    }
    return 0;
}
