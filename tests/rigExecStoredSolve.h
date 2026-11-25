// An aggregate solver's stored result, rests included, read from the
// production program through the row capture the exec-row goldens use.
#ifndef RIGEXEC_TESTS_STORED_SOLVE_H
#define RIGEXEC_TESTS_STORED_SOLVE_H

#include "rigExec/api/bakedProgram.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/api/types.h"
#include "rigExec/internal/bakedExecCrossCheckRows.h"
#include "rigExec/internal/bakedProgramImpl.h"

#include "pxr/usd/usd/stage.h"

#include <string>
#include <vector>

namespace rigExec {

// Compiles a fresh evaluator so the capture is attached before the first
// evaluation; false when the program holds no Solve row for \p solver.
inline bool
RigExecStoredSolve(const PXR_NS::UsdStageRefPtr &stage,
                   const PXR_NS::SdfPath &rigPath,
                   const PXR_NS::SdfPath &solver, PXR_NS::UsdTimeCode time,
                   RigExecPointFrameArray *out)
{
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetSolverGuidesEnabled(true);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors) || !evaluator.GetBakedProgram())
        return false;
    const RigExecBakedProgram &program = *evaluator.GetBakedProgram();
    std::string error;
    if (!RigExecBakedProgramTesting::EnableExecRowCapture(program, &error) ||
        !evaluator.Evaluate(time).valid)
        return false;
    const auto rows = RigExecBakedProgramTesting::ExecCrossCheckRows(program);
    if (!rows)
        return false;
    const std::string key = "Solve " + solver.GetString() + ":";
    for (size_t row = 0; row < rows->Descriptors().size(); ++row) {
        const PXR_NS::VtValue &value = rows->Inputs()[row].expected;
        if (rows->Descriptors()[row].key.compare(0, key.size(), key) == 0 &&
            value.IsHolding<RigExecPointFrameArray>()) {
            *out = value.UncheckedGet<RigExecPointFrameArray>();
            return true;
        }
    }
    return false;
}

} // namespace rigExec

#endif // RIGEXEC_TESTS_STORED_SOLVE_H
