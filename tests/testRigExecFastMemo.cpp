// The effective-memo fast path: byte-identity against the slow key builder
// plus plan coverage over matrix-heavy rigs.
//
// Each rig evaluates its frames twice per schedule mode with
// RIGEXEC_VERIFY_FAST_MEMO on, so every planned memo also runs the slow
// builder and flags any byte or exactness disagreement; the test then
// requires zero mismatches. It also requires every RevisionChunk op to
// carry a plan (chunk memos read revisions only) and at least one planned
// RevisionStatic on the stack-shaped rigs, so a silent loss of the fast
// path fails here rather than as a bench regression.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

int failures = 0;

#define CHECK(cond)                                                  \
    do {                                                             \
        if (!(cond)) {                                               \
            ++failures;                                              \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__,      \
                        #cond);                                      \
        }                                                            \
    } while (0)

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

std::vector<UsdTimeCode>
ParseFrames(const std::string &text)
{
    std::vector<UsdTimeCode> frames;
    for (const std::string &piece : TfStringSplit(text, ",")) {
        if (!piece.empty()) {
            frames.push_back(UsdTimeCode(TfStringToDouble(piece)));
        }
    }
    return frames;
}

struct TestRig {
    const char *stage;
    const char *frames;
};

void
TestOneRig(const std::string &examplesDir, const TestRig &rig,
           bool serial)
{
    const std::string where =
        rig.stage + std::string(serial ? " serial" : " parallel");
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/" + rig.stage);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    TfSetenv("RIGEXEC_BAKED_SCHEDULE", serial ? "serial" : "parallel");
    TfSetenv("RIGEXEC_VERIFY_FAST_MEMO", "1");
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetSolverGuidesEnabled(false);
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    CHECK(compiled);
    if (!compiled) {
        for (const std::string &error : errors) {
            std::printf("    compile: %s\n", error.c_str());
        }
        TfSetenv("RIGEXEC_VERIFY_FAST_MEMO", "0");
        return;
    }
    const std::vector<UsdTimeCode> frames = ParseFrames(rig.frames);
    CHECK(!frames.empty());
    for (int pass = 0; pass < 2; ++pass) {
        for (const UsdTimeCode frame : frames) {
            const RigExecRigPose pose = evaluator.Evaluate(frame);
            CHECK(pose.valid);
        }
    }
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program);
    if (program) {
        const RigExecBakedProgramImpl &B = program->GetStepGraph();
        CHECK(B.verifyFastMemo);
        CHECK(B.fastMemoMismatches == 0);
        if (B.fastMemoMismatches != 0) {
            std::printf("FAIL %s: %zu fast-memo mismatch(es)\n",
                        where.c_str(), B.fastMemoMismatches);
        }
        size_t chunkOps = 0, chunkPlanned = 0;
        size_t staticOps = 0, staticPlanned = 0;
        for (uint32_t c = 0; c < B.opGraph.ops.size(); ++c) {
            const RigExecBakedStep &step =
                B.steps[B.opGraph.ops[c].originalIndex];
            const bool planned =
                c < B.memoFast.size() && B.memoFast[c] != 0;
            if (step.kind == RigExecBakedStepKind::RevisionChunk) {
                ++chunkOps;
                chunkPlanned += planned ? 1 : 0;
            } else if (step.kind ==
                       RigExecBakedStepKind::RevisionStatic) {
                ++staticOps;
                staticPlanned += planned ? 1 : 0;
            }
        }
        CHECK(chunkOps == chunkPlanned);
        if (chunkOps != chunkPlanned) {
            std::printf("FAIL %s: %zu of %zu chunk op(s) planned\n",
                        where.c_str(), chunkPlanned, chunkOps);
        }
        if (staticOps > 0) {
            CHECK(staticPlanned > 0);
        }
        std::printf("  %-34s %2zu chunk op(s) planned, "
                    "%2zu of %2zu static op(s) planned\n",
                    where.c_str(), chunkPlanned, staticPlanned,
                    staticOps);
    }
    TfSetenv("RIGEXEC_VERIFY_FAST_MEMO", "0");
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecFastMemo <examplesDir>\n");
        return 2;
    }
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
#endif
    const std::string examplesDir = argv[1];
    const TestRig rigs[] = {
        {"15_TransformMatrixMover.usda", "1001,1012,1024,1036,1048"},
        {"ArmRig.usda", "1001,1024,1048"},
        {"ArmShotAnim.usda", "1001,1012,1013,1024,1048"},
        {"simple_rig.usd", "1,2,3"},
        {"simple_rig_anim.usd", "0,12,40,51,79,100"},
        {"biped/Biped_body.usda", "1,2,3"},
    };
    for (const TestRig &rig : rigs) {
        TestOneRig(examplesDir, rig, true);
        TestOneRig(examplesDir, rig, false);
    }
    // The matrix mover's own doc example sits beside the examples dir.
    TestOneRig(examplesDir + "/..", {"docs/examples/matrix_mover.usda",
                                            "1001,1008,1016"},
               true);
    TestOneRig(examplesDir + "/..", {"docs/examples/matrix_mover.usda",
                                            "1001,1008,1016"},
               false);
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecFastMemo: all tests passed\n");
    return 0;
}
