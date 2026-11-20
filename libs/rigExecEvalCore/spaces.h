// One space step of a compiled rig, shared by the baked program and .rigexec
// playback: the SpaceExpression provider-op runner (part 0) and the
// ProviderRefresh contextual re-evaluation. Both front ends bind their own
// inputs and publish through their own typed stores; what lives here is the
// kind dispatch, the affine expression sequencing, the missing-input mapping
// and the refresh orchestration, so both make the same decision for the same
// facts. Internal to the evaluator and playback, never an exported API.
//
// A spaces host binds storage and math as host.h requires (nested Math,
// Inputs, State with inputs(), state() and ranges()) and answers the access
// below. Inputs are immutable borrowed views of the compiled op and refresh
// tables; views must not outlive the program. State addresses the existing
// value store, frame versions, commit presence and diagnostics sink by the
// compiled index the adapter already uses. Space steps run serially; ranges()
// is unused and may be null.
//
//   Math     Matrix, Vector, Rotation, Frame and the affine policy
//            AffineMath for RigExecAffineFrameInputsT and
//            RigExecAffineFrameKernel; PointsToMatrix over point arrays and
//            over frames, the commit-gate predicate Usable, and Transform.
//   Inputs   opCount()/op(i)/valueCount() for part 0; refreshCount()/
//            refresh(i) for ProviderRefresh; refreshPath(i) and
//            refreshCarryPath(i, k) spell the diagnostic labels. The op and
//            refresh records stay the adapter's own types and are read
//            through their existing member names only.
//   State    the RigExecRunProviderArithmetic store (ReadMatrix, ReadFrame,
//            ReadScalar, ReadVector, ReadToken, ResetXformStack, Present,
//            Authoritative, Copy, Invalidate, PublishMatrix, PublishFrame),
//            the affine field readers (ReadAffineDouble with float
//            widening, ReadAffineVec3i, ReadAffineBool, ReadAffineInt,
//            ReadAffineString, ReadAffineIntArray, ReadAffineDoubleArray,
//            ReadAffineMatrixArray, each true when the value is present),
//            PublishAffineFailure, the refresh frame and commit accessors,
//            ParentRawMatrix, ReadRefreshFrame and PushDiagnostic.
//
// LocalXform and InterveningXform ops evaluate only on the baked side (they
// compose captured UsdGeom xform orders); here they answer Rejected and the
// runtime reports the output blocked, as it does today. The SpaceExpression
// bridge parts (sampled, external, routed and frame inputs) stay in the
// adapters as input binding; this header runs the expression over them.
#ifndef RIGEXEC_EVAL_CORE_SPACES_H
#define RIGEXEC_EVAL_CORE_SPACES_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "rigExecEvalCore/host.h"
#include "rigExecEvalCore/step.h"
#include "rigExecGraph/providerArithmetic.h"
#include "rigExecMath/affineFrameKernel.h"
#include "rigExecMath/providerRefreshKernel.h"

namespace rigExec {
namespace eval {
namespace spaces {

// Why a space step did not run; the adapter maps each to its own message.
enum class SpaceDetail : int32_t {
    None = 0,
    NoOp = 1,
    OutputRange = 2,
    AffineLayout = 3,
    AffineKind = 4,
    NoRefresh = 5,
};

// One affine input's value into its member: a present, unblocked value of
// the member's type, a float widened for a double; an empty array or any
// other value keeps the member's default.
template <class Host>
void
ReadSpaceAffineField(
    Host &host, uint64_t id, RigExecAffineField field,
    RigExecAffineFrameInputsT<typename Host::Math::AffineMath> *in)
{
    auto &state = host.state();
    using AffineMath = typename Host::Math::AffineMath;
    using Mat4 = typename AffineMath::Mat4;
    using Vec3d = typename AffineMath::Vec3d;
    using Vec3i = typename AffineMath::Vec3i;
    RigExecVisitAffineField(*in, field, [&](auto &out) {
        using T = std::decay_t<decltype(out)>;
        if constexpr (std::is_same_v<T, Mat4>) {
            if (const auto *value = state.ReadMatrix(id))
                out = *value;
        } else if constexpr (std::is_same_v<T, Vec3d>) {
            if (const auto *value = state.ReadVector(id))
                out = *value;
        } else if constexpr (std::is_same_v<T, Vec3i>) {
            Vec3i value;
            if (state.ReadAffineVec3i(id, &value))
                out = value;
        } else if constexpr (std::is_same_v<T, double>) {
            double value = 0.0;
            if (state.ReadAffineDouble(id, &value))
                out = value;
        } else if constexpr (std::is_same_v<T, bool>) {
            bool value = false;
            if (state.ReadAffineBool(id, &value))
                out = value;
        } else if constexpr (std::is_same_v<T, int>) {
            int value = 0;
            if (state.ReadAffineInt(id, &value))
                out = value;
        } else if constexpr (std::is_same_v<T, std::string>) {
            std::string value;
            if (state.ReadAffineString(id, &value))
                out = std::move(value);
        } else if constexpr (std::is_same_v<T, std::vector<int>>) {
            std::vector<int> value;
            if (state.ReadAffineIntArray(id, &value))
                out = std::move(value);
        } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            std::vector<double> value;
            if (state.ReadAffineDoubleArray(id, &value))
                out = std::move(value);
        } else if constexpr (std::is_same_v<T, std::vector<Mat4>>) {
            std::vector<Mat4> value;
            if (state.ReadAffineMatrixArray(id, &value))
                out = std::move(value);
        }
    });
}

// A provider's point frame as a matrix (identity without one).
template <class Host>
typename Host::Math::AffineMath::Mat4
SpaceAffineFrameOf(Host &host, uint64_t id)
{
    using AffineMath = typename Host::Math::AffineMath;
    using Kernel = RigExecAffineFrameKernel<AffineMath>;
    using Mat4 = typename AffineMath::Mat4;
    const auto *frame = host.state().ReadFrame(id);
    if (!frame)
        return Mat4(1);
    return Kernel::FrameMatrix(frame->points[0], frame->points[1],
                               frame->points[2], frame->points[3]);
}

// An affine frame expression: the op's inputs in its layout's order through
// the shared kernel. A failed expression publishes an unblocked empty value
// carrying the kernel's reason, as both adapters do today.
template <class Host, class Op>
StepResult
RunSpaceAffineOp(Host &host, const Op &op)
{
    using AffineMath = typename Host::Math::AffineMath;
    using Kernel = RigExecAffineFrameKernel<AffineMath>;
    using Inputs = RigExecAffineFrameInputsT<AffineMath>;
    using Mat4 = typename AffineMath::Mat4;
    if (op.affineKind >= RigExecAffineFrameTypeCount)
        return StepResult{Outcome::Invalid,
                          static_cast<int32_t>(SpaceDetail::AffineKind)};
    if (!RigExecAffineFrameInputsMatch(op.affineKind, op.inputs.size(),
                                       op.affineTargets))
        return StepResult{Outcome::Invalid,
                          static_cast<int32_t>(SpaceDetail::AffineLayout)};
    const RigExecAffineFrameType &type = RigExecAffineFrameTypes[op.affineKind];
    const auto id = [&](size_t at) -> uint64_t {
        return at < op.inputs.size() ? uint64_t(op.inputs[at]) : UINT64_MAX;
    };
    Inputs in;
    size_t at = 0;
    for (uint32_t k = 0; k < type.attributeCount; ++k)
        ReadSpaceAffineField(host, id(at++), type.attributes[k].field, &in);
    for (uint32_t k = 0; k < type.relationCount; ++k) {
        const Mat4 frame = SpaceAffineFrameOf(host, id(at++));
        RigExecVisitAffineField(in, type.relations[k].field, [&](auto &out) {
            if constexpr (std::is_same_v<std::decay_t<decltype(out)>, Mat4>)
                out = frame;
        });
    }
    if (type.targets) {
        for (uint32_t k = 0; k < op.affineTargets; ++k)
            in.targets.push_back(SpaceAffineFrameOf(host, id(at++)));
        while (at < op.inputs.size())
            in.targetObjects.push_back(SpaceAffineFrameOf(host, id(at++)));
    }
    Mat4 result(1);
    const char *failure = nullptr;
    if (!Kernel::Compute(op.affineKind, in, &result, &failure)) {
        host.state().PublishAffineFailure(
            uint64_t(op.output),
            std::string("affine frame expression failed: ") + failure);
        return StepResult{Outcome::Applied,
                          static_cast<int32_t>(SpaceDetail::None)};
    }
    host.state().PublishMatrix(uint64_t(op.output), result, false);
    return StepResult{Outcome::Applied,
                      static_cast<int32_t>(SpaceDetail::None)};
}

// SpaceExpression part 0: the portable provider op the step names. Unknown
// kinds answer Rejected; the adapters map that to their existing fallback.
template <class Host>
StepResult
RunSpaceOp(Host &host, const StepRef &step)
{
    const auto &inputs = host.inputs();
    if (step.object < 0 || size_t(step.object) >= inputs.opCount())
        return StepResult{Outcome::Invalid,
                          static_cast<int32_t>(SpaceDetail::NoOp)};
    const auto &op = inputs.op(size_t(step.object));
    if (uint64_t(op.output) >= inputs.valueCount())
        return StepResult{Outcome::Invalid,
                          static_cast<int32_t>(SpaceDetail::OutputRange)};
    if (op.kind == RigExecProviderOpKind::AffineFrame)
        return RunSpaceAffineOp(host, op);
    if (!RigExecRunProviderArithmetic<typename Host::Math>(op, host.state()))
        return StepResult{Outcome::Rejected,
                          static_cast<int32_t>(SpaceDetail::None)};
    return StepResult{Outcome::Applied,
                      static_cast<int32_t>(SpaceDetail::None)};
}

// ProviderRefresh: the constrained check, the carry capture with its parent
// blocks, the shared frame kernel, the write-back and the outcome's single
// diagnostic, in the order both adapters run them today.
template <class Host>
StepResult
RunSpaceRefresh(Host &host, const StepRef &step)
{
    const auto &inputs = host.inputs();
    auto &state = host.state();
    if (step.object < 0 || size_t(step.object) >= inputs.refreshCount())
        return StepResult{Outcome::Invalid,
                          static_cast<int32_t>(SpaceDetail::NoRefresh)};
    const size_t index = size_t(step.object);
    const auto &refresh = inputs.refresh(index);
    bool constrained = false;
    for (const auto &prior : refresh.priorConstraints)
        constrained =
            constrained || (!state.CommitAbandoned(prior.first) &&
                            state.CommitPresent(prior.first, prior.second));
    using Math = typename Host::Math;
    using Frame = typename Math::Frame;
    using Matrix = typename Math::Matrix;
    const Matrix identity(1);
    for (size_t k = 0; k < refresh.carries.size(); ++k) {
        const auto &carry = refresh.carries[k];
        state.RefreshBaseInputs(index)[k] = state.BaseFrame(carry.baseRead);
        state.RefreshFinInputs(index)[k] = state.FinFrame(carry.finRead);
        state.RefreshBlocked(index)[k] = 0;
        for (const auto slot : carry.blockingSlots)
            if (state.ParentRawMatrix(slot) != identity)
                state.RefreshBlocked(index)[k] = 1;
    }
    Frame rawStorage, currentStorage;
    size_t failed = 0;
    const auto outcome = RigExecRefreshProviderFrames<Math>(
        state.BaseFrame(refresh.baseRead), state.FinFrame(refresh.finRead),
        state.ReadRefreshFrame(uint64_t(refresh.baseValue), &rawStorage),
        state.ReadRefreshFrame(uint64_t(refresh.currentValue), &currentStorage),
        constrained, state.RefreshBaseInputs(index),
        state.RefreshFinInputs(index), state.RefreshBlocked(index),
        refresh.carries.size(), state.MutableBaseFrame(refresh.baseWrite),
        state.MutableFinFrame(refresh.finWrite),
        state.RefreshBaseOutputs(index), state.RefreshFinOutputs(index),
        &failed);
    for (size_t k = 0; k < refresh.carries.size(); ++k) {
        const auto &carry = refresh.carries[k];
        *state.MutableBaseFrame(carry.baseWrite) =
            state.RefreshBaseOutputs(index)[k];
        *state.MutableFinFrame(carry.finWrite) =
            state.RefreshFinOutputs(index)[k];
    }
    const auto path = inputs.refreshPath(index);
    if (outcome == RigExecProviderRefreshOutcome::MissingBase)
        state.PushDiagnostic(
            std::string("connected base pose input incomplete: ") +
            std::string(path));
    else if (outcome == RigExecProviderRefreshOutcome::MissingCurrent)
        state.PushDiagnostic(
            std::string("connected final pose input incomplete: ") +
            std::string(path));
    else if (outcome == RigExecProviderRefreshOutcome::InvalidCurrent)
        state.PushDiagnostic(std::string(path) +
                             " produced an invalid or degenerate frame for " +
                             std::string(path) + "; constraint passed through");
    else if (outcome == RigExecProviderRefreshOutcome::SingularCurrent)
        state.PushDiagnostic(
            std::string(path) +
            " produced a singular hierarchy delta; constraint passed through");
    else if (outcome == RigExecProviderRefreshOutcome::InvalidDescendant)
        state.PushDiagnostic(
            std::string(path) +
            " could not propagate its pose revision through " +
            std::string(inputs.refreshCarryPath(index, failed)) +
            "; constraint passed through");
    else if (outcome ==
             RigExecProviderRefreshOutcome::InvalidTransformedDescendant)
        state.PushDiagnostic(
            std::string(path) + " produced an invalid descendant frame for " +
            std::string(inputs.refreshCarryPath(index, failed)) +
            "; constraint passed through");
    return StepResult{Outcome::Applied,
                      static_cast<int32_t>(SpaceDetail::None)};
}

} // namespace spaces

// The space kinds behind one entry: SpaceExpression part 0 runs the portable
// provider op the step names (bridge parts stay adapter-local input
// binding), ProviderRefresh re-evaluates the named context. Adapters route
// the step here and map the outcome to their existing stores and messages.
template <class Host>
StepResult
RunValueStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value,
                  "a spaces host binds Math, Inputs and State with inputs(), "
                  "state() and ranges()");
    if (step.kind == uint32_t(StepKind::SpaceExpression))
        return spaces::RunSpaceOp(host, step);
    if (step.kind == uint32_t(StepKind::ProviderRefresh))
        return spaces::RunSpaceRefresh(host, step);
    return StepResult{Outcome::Invalid,
                      static_cast<int32_t>(spaces::SpaceDetail::None)};
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_SPACES_H
