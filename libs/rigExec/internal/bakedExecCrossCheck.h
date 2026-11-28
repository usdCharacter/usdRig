#ifndef RIGEXEC_BAKED_EXEC_CROSS_CHECK_H
#define RIGEXEC_BAKED_EXEC_CROSS_CHECK_H

#include "rigExec/internal/valueAddress.h"

#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace rigExec {

enum class RigExecExecCheckSkip {
    None,
    NoEquivalent,
    ArrayInput,
    Unusable,
    /// The oracle could not resolve a name it needs. Reported as a failure,
    /// never a skip: a missing name is a bug in the row's table.
    MissingName
};

/// Diagnostic spelling of a skip reason. Row keys and the step-kind fallback
/// read it here instead of re-spelling the label.
namespace RigExecExecCheckSkipText {
inline constexpr const char *const noEquivalent = "NoEquivalent";
}

struct RigExecExecCheckDescriptor {
    std::string key;
    uint32_t kind = 0;
    /// A row with no public equivalent must explicitly skip.
    bool skipOnly = false;
    RigExecValueAddress address;
    /// A required override names a bound value, never the op's output.
    std::vector<RigExecValueAddress> requiredOverrides;
    /// Optional pure conversion from a reference result to the stored op type.
    std::function<bool(const VtValue &, VtValue *)> project;
    using Reference = std::function<bool(VtValue *, std::string *)>;
    /// Owning-thread acquisition of independent ORIGINAL input context.
    /// The returned numerical witness owns its inputs and performs no stage reads.
    std::function<bool(const UsdStageRefPtr &, UsdTimeCode,
                       const std::vector<RigExecValueOverride> &, Reference *,
                       std::string *)>
        acquireReference;
};
struct RigExecExecCheckInput {
    VtValue expected;
    std::vector<RigExecValueOverride> overrides;
    RigExecExecCheckSkip skip = RigExecExecCheckSkip::None;
};

/// Every capture and post-capture classification routes its skip here. A row
/// whose name could not be interned is fatal: an ordinary reason may not
/// replace it and hide the bug as a counted skip.
inline void
RigExecExecCheckSetSkip(RigExecExecCheckInput &input, RigExecExecCheckSkip skip)
{
    if (input.skip == RigExecExecCheckSkip::MissingName)
        return;
    input.skip = skip;
}
struct RigExecExecCrossCheckReport {
    size_t checked = 0, failed = 0, dropped = 0;
    std::map<uint32_t, size_t> checkedByKind;
    std::map<RigExecExecCheckSkip, size_t> skipped;
    std::vector<std::string> diagnostics;
    bool Passed() const
    {
        return failed == 0 && dropped == 0;
    }
};

/// Owning-thread verification only. A row with an independent ORIGINAL
/// constraint witness is judged against it; any other row has no judge and
/// is counted as NoEquivalent (the exec-row goldens pin its value). Inputs
/// are immutable copies of the values the operation read.
class RigExecBakedExecCrossCheck {
public:
    explicit RigExecBakedExecCrossCheck(const UsdStageRefPtr &stage)
        : _stage(stage)
    {
    }
    bool Add(RigExecExecCheckDescriptor row, std::string *error = nullptr);
    RigExecExecCrossCheckReport
    Evaluate(UsdTimeCode time,
             const std::vector<RigExecExecCheckInput> &inputs);
    size_t Size() const
    {
        return _rows.size();
    }

private:
    UsdStageRefPtr _stage;
    std::vector<RigExecExecCheckDescriptor> _rows;
};
} // namespace rigExec
#endif
