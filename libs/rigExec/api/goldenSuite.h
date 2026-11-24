#ifndef RIGEXEC_GOLDEN_SUITE_H
#define RIGEXEC_GOLDEN_SUITE_H

#include "pxr/usd/sdf/path.h"
#include <memory>
#include <string>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
struct RigExecRigPose;

/// Optional owning-thread suite capture. Each evaluator owns its file and
/// buffer; the disabled path allocates nothing and records nothing.
/// A process-completion index checks total evaluator/generation counts and
/// capture inventory; each evaluator's file checks its own exact visit order.
/// The recorder lives in the test oracle library. Create returns null when
/// the suite is disabled, and throws std::runtime_error if enabled without
/// rigExecOracle registration. Record is virtual so the shared library does not
/// contain the capture.
class RigExecGoldenSuiteObserver {
public:
    static std::unique_ptr<RigExecGoldenSuiteObserver>
    Create(const SdfPath &rigPath);
    virtual ~RigExecGoldenSuiteObserver() = default;
    virtual void Record(const RigExecRigPose &pose) = 0;

protected:
    RigExecGoldenSuiteObserver() = default;
};
/// Completes the golden suite after every evaluator has closed: checks the
/// process index and inventory. Idempotent; does nothing when the suite is
/// off, and throws std::runtime_error when it is on without rigExecOracle.
void RigExecFinalizeGoldenSuite();
} // namespace rigExec
#endif
