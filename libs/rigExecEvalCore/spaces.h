// The provider program's affine frame sequencing and input classes, shared
// by the AffineFrame and ProviderArithmetic ops and the hosts that fill
// their typed input lists. Header-only, free of OpenUSD; internal to the
// evaluator, playback and the op library, never an exported API.
//
// RunSpaceAffineOp's host binds Math::AffineMath and answers state() with
// the reads below; ReadSpaceAffineField maps one typed value into its
// member and a missing one keeps the member's default.
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
#include "rigExecGraph/providerInputs.h"
#include "rigExecMath/affineFrameKernel.h"

namespace rigExec {
namespace eval {
namespace spaces {

// Why an affine expression did not run.
enum class SpaceDetail : int32_t {
    None = 0,
    AffineLayout = 3,
    AffineKind = 4,
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
// the shared kernel. A failed expression hands the kernel's reason to
// PublishAffineFailure.
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

} // namespace spaces
} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_SPACES_H
