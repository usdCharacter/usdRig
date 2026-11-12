#ifndef RIGEXEC_BAKED_EXEC_CROSS_CHECK_ROWS_H
#define RIGEXEC_BAKED_EXEC_CROSS_CHECK_ROWS_H

#include "bakedExecCrossCheck.h"
#include "rigExecMath/pointFrame.h"
#include <memory>

namespace rigExec {
struct RigExecBakedProgramImpl;
enum class RigExecBakedStepKind;
struct RigExecExecArrayWitness {
    SdfPath path;
    VtValue value;
    bool atDefault = false;
};

/// Row names resolve through one interned table built at load, declared with
/// the row tables so the row test can drive the failure path on real capture
/// code instead of fabricating row state.
const TfToken *RigExecExecCheckToken(const char *name);
/// Build runs on the owning thread with an error channel: an uninterned name
/// fails the build there instead of spelling an empty property.
bool RigExecExecCheckResolve(const char *name, TfToken &out,
                             std::string *error);
/// Capture-lane lookup. A miss marks the row fatal and returns false; it never
/// interns text or posts a diagnostic.
bool RigExecExecCheckCaptureName(const char *name, RigExecExecCheckInput &input,
                                 TfToken &out);
/// The classification a mirrored row receives after its step runs.
void RigExecExecCheckApplyAfterStep(RigExecExecCheckInput &input,
                                    bool frameMatrixInvalid,
                                    bool emptySolveFrames);

/// Optional live-program instrumentation. Preparation and evaluation belong
/// to the owning thread; capture copies only program values and each operation
/// writes its own rows. Frozen programs do not carry this state.
class RigExecBakedExecCheckRows {
    struct _Tag {
        explicit _Tag(int)
        {
        }
    };

public:
    static std::shared_ptr<RigExecBakedExecCheckRows>
    Build(const RigExecBakedProgramImpl &, std::string *error = nullptr);
    // Public for std::make_shared; the gate is that only a member or friend
    // can build the non-aggregate _Tag, which brace-init cannot synthesize.
    RigExecBakedExecCheckRows(_Tag, const UsdStageRefPtr &stage)
        : RigExecBakedExecCheckRows(stage)
    {
    }
    // Virtual so the evaluator can call the test oracle library. The vtable
    // lives with the oracle objects, not in the shared library.
    virtual ~RigExecBakedExecCheckRows();
    virtual void BeforeStep(const RigExecBakedProgramImpl &, size_t step);
    virtual void BeforeSlot(const RigExecBakedProgramImpl &,
                            RigExecBakedStepKind, int groupBegin, int slot);
    virtual void AfterStep(const RigExecBakedProgramImpl &, size_t step);
    /// Checks-only actual candidate observation; never used as reference input.
    virtual void ObserveConstraintCandidate(int walk,
                                            const RigExecPointFrame &);
    RigExecExecCrossCheckReport Evaluate(UsdTimeCode);
    const std::vector<RigExecExecCheckDescriptor> &Descriptors() const
    {
        return _descriptors;
    }
    /// Immutable capture inspection and intentional witness mutations for tests.
    const std::vector<RigExecExecCheckInput> &Inputs() const
    {
        return _inputs;
    }
    std::vector<RigExecExecCheckInput> &TestingInputs()
    {
        return _inputs;
    }
    bool WasChecked(size_t row) const
    {
        return row < _lastChecked.size() && _lastChecked[row];
    }

private:
    struct Binding {
        size_t step = 0;
        int slot = -1;
        bool perSlot = false;
        std::function<RigExecExecCheckInput(const RigExecBakedProgramImpl &)>
            capture;
        std::function<VtValue(const RigExecBakedProgramImpl &)> result;
        std::function<std::vector<RigExecExecArrayWitness>(
            const RigExecBakedProgramImpl &)>
            arrays;
    };
    explicit RigExecBakedExecCheckRows(const UsdStageRefPtr &stage)
        : _check(stage), _stage(stage)
    {
    }
    bool Add(RigExecExecCheckDescriptor, Binding,
             const RigExecBakedProgramImpl &, std::string *error);
    RigExecBakedExecCrossCheck _check;
    std::vector<Binding> _bindings;
    std::vector<RigExecExecCheckDescriptor> _descriptors;
    std::vector<RigExecExecCheckInput> _inputs;
    std::vector<char> _captured, _candidateProduced;
    std::vector<RigExecPointFrame> _constraintCandidates;
    std::map<int, size_t> _constraintRows;
    std::vector<char> _lastChecked;
    UsdStageRefPtr _stage;
    std::vector<std::vector<RigExecExecArrayWitness>> _stageArrays;
    std::vector<std::vector<size_t>> _byStep;
    std::map<std::pair<uint32_t, int>, std::vector<size_t>> _bySlot;
};
} // namespace rigExec
#endif
