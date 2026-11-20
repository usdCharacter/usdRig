// Evaluate one scheduled property-chain part over borrowed inputs and versions.
#ifndef RIGEXEC_EVAL_CORE_PROPERTIES_H
#define RIGEXEC_EVAL_CORE_PROPERTIES_H

#include "rigExecEvalCore/host.h"
#include "rigExecMath/propertyMathKernel.h"

#include <cmath>
#include <string>
#include <type_traits>
#include <vector>

namespace rigExec {
namespace eval {
namespace properties {

using ValueType = RigExecWirePropertyValueType;
enum class Operand { Value, Minimum, Maximum };
enum class BaseRead { Present, Missing, NoTarget };
enum class Detail : int32_t {
    NoTarget = 1,
    MissingBase,
    NonFiniteBase,
    Disabled,
    WeightField,
    DefaultWeight,
    UnusableInputs,
    NonFiniteResult
};

struct WeightFieldView {
    bool ok;
    size_t count;
    const float *values;
    const std::string &error;
};

template <class T> struct ArrayView {
    const T *elements = nullptr;
    size_t count = 0;
    const T *data() const
    {
        return elements;
    }
    size_t size() const
    {
        return count;
    }
};

template <class Math, class T>
bool
IsFinite(const T &value)
{
    if constexpr (std::is_arithmetic_v<T>) {
        // Double chains compute in float, including their base check.
        return std::isfinite(float(value));
    } else if constexpr (std::is_same_v<T, typename Math::Vec3>) {
        return std::isfinite(value[0]) && std::isfinite(value[1]) &&
               std::isfinite(value[2]);
    } else {
        for (size_t r = 0; r < 4; ++r)
            for (size_t c = 0; c < 4; ++c)
                if (!std::isfinite(value[r][c]))
                    return false;
        return true;
    }
}

template <class Math, class Revision, class T>
bool
Apply(const Revision &revision, const T &in, float envelope, T &out)
{
    if (!revision.opValid())
        return false;
    using MathValue = std::conditional_t<std::is_same_v<T, double>, float, T>;
    const auto op = revision.op();
    const auto value = revision.template read<MathValue>(Operand::Value);
    if constexpr (std::is_same_v<T, typename Math::Matrix>) {
        return IsFinite<Math>(value) &&
               RigExecApplyMatrixMathKernel(in, op, value, envelope, &out);
    } else {
        RigExecPropertyMathKernelParams<MathValue, typename Math::Key> params;
        params.op = op;
        params.value = value;
        params.min = revision.template read<MathValue>(Operand::Minimum);
        params.max = revision.template read<MathValue>(Operand::Maximum);
        params.weight = envelope;
        if (!IsFinite<Math>(value) || !IsFinite<Math>(params.min) ||
            !IsFinite<Math>(params.max))
            return false;
        if constexpr (std::is_same_v<MathValue, float>) {
            if (op == RigExecPropertyOp::Curve) {
                const auto keys = revision.keys();
                const auto tangents = revision.tangents();
                if (!keys.size() || !revision.keysValid(keys) ||
                    (tangents.size() && tangents.size() != keys.size()))
                    return false;
                params.keys = keys.data();
                params.keyCount = keys.size();
                params.tangents = tangents.size() ? tangents.data() : nullptr;
                params.tangentCount = tangents.size();
            } else if (op == RigExecPropertyOp::Path) {
                const auto points = revision.pathPoints();
                const auto &path = revision.pathSettings();
                static_assert(sizeof(typename Math::Vec3) == 3 * sizeof(float),
                              "path points are packed floats");
                params.path = {points.size() ? points.data()->data() : nullptr,
                               points.size(), path.basis, path.pinned,
                               path.planar};
                params.pathSettings = path;
                if (!path.valid || !RigExecPathCurveValid(params.path))
                    return false;
            }
            out = T(RigExecApplyFloatMathKernel(float(in), params));
        } else {
            out = RigExecApplyVec3fMathKernel(in, params);
        }
        return true;
    }
}

template <class Math, class Revision, class T>
StepResult
Revise(const Revision &revision, T &value, std::vector<std::string> &lines)
{
    if (!revision.exists())
        return {Outcome::Applied, kNoStepDetail};
    const auto reject = [&](Detail detail, const std::string &reason) {
        lines.push_back(revision.label() + ": " + reason +
                        "; revision passed through");
        return StepResult{Outcome::Rejected, int32_t(detail)};
    };
    if (!revision.enabled())
        return reject(Detail::Disabled, "disabled");
    float envelope;
    if (revision.hasWeightField()) {
        const auto field = revision.weightField();
        if (!field.ok || field.count != 1)
            return reject(Detail::WeightField, field.error);
        envelope = field.values[0];
    } else {
        envelope = revision.defaultWeight();
        if (!std::isfinite(envelope) || envelope < 0.0f || envelope > 1.0f)
            return reject(Detail::DefaultWeight,
                          "inputs:defaultWeight must be finite and in [0, 1]");
    }
    T next = value;
    if (!Apply<Math>(revision, value, envelope, next))
        return reject(Detail::UnusableInputs, "inputs unusable");
    if (!IsFinite<Math>(next))
        return reject(Detail::NonFiniteResult, "produced a non-finite value");
    value = next;
    return {Outcome::Applied, kNoStepDetail};
}

// The adapter only encodes the typed store and its change notifications;
// EvalCore decides which producer version is read and what the part writes.
template <class T, class Host>
StepResult
RunPropertyPart(Host &host, const StepRef &step)
{
    using Math = typename Host::Math;
    const auto &inputs = host.inputs();
    auto &state = host.state();
    const size_t chain = size_t(step.object);
    const size_t part = size_t(step.part);
    T value{};
    bool valid = false;
    const T *payload = &value;
    StepResult result{Outcome::Applied, kNoStepDetail};
    if (part == 0) {
        const auto reject = [&](Detail detail, const char *reason) {
            state.lines.push_back(inputs.baseLabel(chain) + ": " + reason +
                                  "; chain skipped");
            result = {Outcome::Rejected, int32_t(detail)};
        };
        switch (inputs.readBase(chain, value)) {
        case BaseRead::NoTarget:
            payload = nullptr;
            reject(Detail::NoTarget, "target attribute disappeared");
            break;
        case BaseRead::Missing:
            payload = nullptr;
            reject(Detail::MissingBase, "target has no authored value");
            break;
        case BaseRead::Present:
            valid = IsFinite<Math>(value);
            if (!valid)
                reject(Detail::NonFiniteBase, "authored base is not finite");
            break;
        }
        state.setChainValid(chain, valid);
    } else {
        value = state.template version<T>(step.slot - 1);
        valid = state.predecessorValid(chain, step.slot - 1);
        if (valid) {
            const auto revision = inputs.revision(chain, part - 1);
            revision.appendInputDiagnostics(state.lines);
            result = Revise<Math>(revision, value, state.lines);
        } else {
            result = {Outcome::Rejected, kNoStepDetail};
        }
    }
    state.storeVersion(step.slot, payload, valid);
    inputs.forEachRecord(chain, [&](size_t index, const auto &record) {
        if (record.applied != part)
            return;
        const bool recordValid = valid && !state.recordStoodAside(index);
        if (!payload) {
            state.storeVersion(record.version, payload, false);
        } else if constexpr (std::is_same_v<T, float>) {
            if (record.consumerType == ValueType::Double) {
                const double converted = *payload;
                state.storeVersion(record.version, &converted, recordValid);
            } else {
                state.storeVersion(record.version, payload, recordValid);
            }
        } else if constexpr (std::is_same_v<T, double>) {
            if (record.consumerType == ValueType::Float) {
                const float converted = float(*payload);
                state.storeVersion(record.version, &converted, recordValid);
            } else {
                state.storeVersion(record.version, payload, recordValid);
            }
        } else {
            state.storeVersion(record.version, payload, recordValid);
        }
    });
    return result;
}

} // namespace properties

template <class Host>
StepResult
RunValueStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value,
                  "property steps require a bound host");
    const auto &inputs = host.inputs();
    if (step.kind != uint32_t(StepKind::PropertyRevision) || step.object < 0 ||
        step.part < 0 || size_t(step.object) >= inputs.chainCount() ||
        size_t(step.part) > inputs.revisionCount(size_t(step.object)) ||
        step.slot !=
            inputs.versionBase(size_t(step.object)) + size_t(step.part))
        return {Outcome::Invalid, kNoStepDetail};
    using Math = typename Host::Math;
    switch (inputs.valueType(size_t(step.object))) {
    case properties::ValueType::Float:
        return properties::RunPropertyPart<float>(host, step);
    case properties::ValueType::Double:
        return properties::RunPropertyPart<double>(host, step);
    case properties::ValueType::Matrix4d:
        return properties::RunPropertyPart<typename Math::Matrix>(host, step);
    case properties::ValueType::Vec3f:
        return properties::RunPropertyPart<typename Math::Vec3>(host, step);
    }
    return {Outcome::Invalid, kNoStepDetail};
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_PROPERTIES_H
