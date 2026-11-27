#include "rigExecSchema/schemaNames.h"
#include "spaces.h"
#include "poseInternal.h"
#include "rigExecMath/affineMath.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecEvalCore/spaces.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/array.h"
#include "rigExecStdTypes/stdTypes.h"
#include <algorithm>
#include <cstring>
#include <type_traits>
namespace rigExec {
namespace {
struct Store {
    std::vector<RigExecProviderPlainState> &values;
    explicit Store(std::vector<RigExecProviderPlainState> &v) : values(v)
    {
    }
    void Invalidate(uint64_t output, uint64_t source)
    {
        const bool blocked = source < values.size() && values[source].blocked;
        auto &out = values[output];
        out.value = std::monostate{};
        out.initialized = true;
        out.blocked = blocked;
        out.authoritative = false;
        out.count = 0;
        out.error = "provider input unavailable";
    }
    void PublishMatrix(uint64_t id, const RrMat4d &matrix, bool authority)
    {
        auto &out = values[id];
        std::array<double, 16> value;
        std::memcpy(value.data(), matrix._mtx, sizeof(double) * 16);
        out.value = value;
        out.initialized = true;
        out.blocked = false;
        out.authoritative = authority;
        out.count = 1;
        out.error.clear();
    }
    void PublishFrame(uint64_t id, const RrPointFrame &frame)
    {
        RigExecProviderPlainFrame value;
        for (size_t p = 0; p < 4; ++p)
            for (size_t a = 0; a < 3; ++a)
                value.points[p][a] = frame.points[p][a];
        value.flags = frame.flags;
        auto &out = values[id];
        out.value = value;
        out.initialized = true;
        out.blocked = false;
        out.authoritative = false;
        out.count = 1;
        out.error.clear();
    }
};
} // namespace
bool
RrReadCrossDomain(const RrProgram *program, int32_t index, RrWireValue *out)
{
    if (index < 0 || size_t(index) >= program->file->crossDomainReads.size())
        return false;
    const auto &read = program->file->crossDomainReads[size_t(index)];
    if (read.kind == fb::CrossDomainReadKind::PointElement) {
        RrVec3f point;
        if (!RrReadCrossDomainPoint(program, read, &point))
            return false;
        out->tag = RigExecWireInputTag::Vec3f;
        out->vec3f = {point[0], point[1], point[2]};
        return true;
    }
    if (read.kind == fb::CrossDomainReadKind::PoseFrame) {
        const auto &frames =
            read.baseFrame ? program->store.base : program->store.fin;
        const std::array<RrVec3d, 4> identity = {
            RrVec3d(0), RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
        for (auto version : read.frames) {
            const auto &frame = frames[version];
            if (!frame.IsValid() || frame.IsDegenerate())
                continue;
            RrMat4d matrix(1);
            if (!RrPointsToMatrix(identity, frame, &matrix))
                continue;
            out->tag = RigExecWireInputTag::Matrix4d;
            std::memcpy(out->matrix.data(), matrix._mtx, sizeof(double) * 16);
            return true;
        }
        return false;
    }
    if (read.kind == fb::CrossDomainReadKind::PropertyResult) {
        if (!program->store.propertyVersionValid[read.propertyVersion])
            return false;
        const auto &v = program->store.propertyVersions[read.propertyVersion];
        switch (v.tag) {
        case RrPropertyValue::Tag::Float: {
            out->tag = RigExecWireInputTag::Float;
            uint32_t bits;
            std::memcpy(&bits, &v.f32, sizeof(bits));
            out->bits = bits;
            break;
        }
        case RrPropertyValue::Tag::Double:
            out->tag = RigExecWireInputTag::Double;
            std::memcpy(&out->bits, &v.f64, sizeof(v.f64));
            break;
        case RrPropertyValue::Tag::Vec3f:
            out->tag = RigExecWireInputTag::Vec3f;
            out->vec3f = {v.vec[0], v.vec[1], v.vec[2]};
            break;
        case RrPropertyValue::Tag::Matrix4d:
            out->tag = RigExecWireInputTag::Matrix4d;
            std::memcpy(out->matrix.data(), v.matrix._mtx, sizeof(double) * 16);
            break;
        }
        return true;
    }
    if (read.spaceValue < 0)
        return false;
    const auto &state = program->store.providerValues[size_t(read.spaceValue)];
    if (!state.initialized || state.blocked)
        return false;
    if (const auto *matrix =
            std::get_if<std::array<double, 16>>(&state.value)) {
        out->tag = RigExecWireInputTag::Matrix4d;
        out->matrix = *matrix;
        return true;
    }
    if (const auto *value = std::get_if<double>(&state.value)) {
        out->tag = RigExecWireInputTag::Double;
        std::memcpy(&out->bits, value, sizeof(double));
        return true;
    }
    if (const auto *value = std::get_if<float>(&state.value)) {
        out->tag = RigExecWireInputTag::Float;
        uint32_t bits;
        std::memcpy(&bits, value, sizeof(float));
        out->bits = bits;
        return true;
    }
    if (const auto *value = std::get_if<std::array<double, 3>>(&state.value)) {
        out->tag = RigExecWireInputTag::Vec3d;
        out->vec3d = *value;
        return true;
    }
    if (const auto *value = std::get_if<std::array<float, 3>>(&state.value)) {
        out->tag = RigExecWireInputTag::Vec3f;
        out->vec3f = *value;
        return true;
    }
    if (const auto *value = std::get_if<std::array<int32_t, 3>>(&state.value)) {
        out->tag = RigExecWireInputTag::Vec3i;
        out->vec3i = *value;
        return true;
    }
    if (const auto *value = std::get_if<bool>(&state.value)) {
        out->tag = RigExecWireInputTag::Bool;
        out->bits = *value ? 1 : 0;
        return true;
    }
    if (const auto *value = std::get_if<int32_t>(&state.value)) {
        out->tag = RigExecWireInputTag::Int;
        out->bits = uint32_t(*value);
        return true;
    }
    if (const auto *value = std::get_if<std::string>(&state.value)) {
        // Token ownership is established at Open/sampling, never by a consumer.
        const auto found = program->inputState.tokenIds.find(*value);
        if (found == program->inputState.tokenIds.end())
            return false;
        out->tag = RigExecWireInputTag::Token;
        out->bits = found->second;
        return true;
    }
    if (const auto *value =
            std::get_if<RigExecProviderPlainFrame>(&state.value)) {
        RrPointFrame frame;
        for (size_t k = 0; k < 4; ++k)
            frame.points[k] = RrVec3d(value->points[k][0], value->points[k][1],
                                      value->points[k][2]);
        frame.flags = value->flags;
        const std::array<RrVec3d, 4> identity = {
            RrVec3d(0), RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
        RrMat4d matrix(1);
        if (!frame.IsValid() || frame.IsDegenerate() ||
            !RrPointsToMatrix(identity, frame, &matrix))
            return false;
        out->tag = RigExecWireInputTag::Matrix4d;
        std::memcpy(out->matrix.data(), matrix._mtx, sizeof(double) * 16);
        return true;
    }
    return false;
}

bool
RrOpenProviderProgram(RrProgram *program, std::string *error)
{
    if (!program->file)
        return true;
    using T = eval::OpOutputTarget;
    program->providerAnswerOfNode.clear();
    program->providerOpOfNode.clear();
    program->refreshOfNode.clear();
    if (const auto &graph = program->file->opGraph) {
        const size_t nodes = graph->nodes.size();
        program->providerAnswerOfNode.assign(nodes, -1);
        program->providerOpOfNode.assign(nodes, -1);
        program->refreshOfNode.assign(nodes, -1);
        for (const auto &output : graph->hostOutputs) {
            if (!output || output->node >= nodes)
                continue;
            const auto set = [&](std::vector<int64_t> &of) {
                int64_t &at = of[size_t(output->node)];
                if (at < 0)
                    at = int64_t(output->object);
            };
            if (output->target == uint8_t(T::ProviderAnswer))
                set(program->providerAnswerOfNode);
            else if (output->target == uint8_t(T::ProviderValueMatrix) ||
                     output->target == uint8_t(T::ProviderValueFrame))
                set(program->providerOpOfNode);
            else if (output->target == uint8_t(T::RefreshBase))
                set(program->refreshOfNode);
        }
    }
    if (!program->file->providerProgram)
        return true;
    const auto &wire = *program->file->providerProgram;
    auto &plain = program->providerProgram;
    plain.valueKeys = wire.valueKeys;
    plain.leaves = wire.leaves;
    const auto internToken = [&](const std::string &text) {
        auto &state = program->inputState;
        if (state.tokenIds.find(text) != state.tokenIds.end())
            return;
        const uint32_t id =
            state.extraTokenBase + uint32_t(state.extraTokens.size());
        state.extraTokens.push_back(text);
        state.tokenIds.emplace(text, id);
    };
    for (const auto &row : wire.defaults) {
        RigExecProviderPlainState out;
        out.initialized = row.initialized;
        out.blocked = row.blocked;
        out.authoritative = row.authoritative;
        out.count = row.count;
        out.error = row.error;
        switch (row.kind) {
        case fb::ProviderValueKind::Empty:
            break;
        case fb::ProviderValueKind::Double:
            out.value = *row.scalarDouble;
            break;
        case fb::ProviderValueKind::Float:
            out.value = *row.scalarFloat;
            break;
        case fb::ProviderValueKind::Vector:
            out.value = *row.vector;
            break;
        case fb::ProviderValueKind::Matrix:
            out.value = *row.matrix;
            break;
        case fb::ProviderValueKind::Token:
            internToken(row.token);
            out.value = row.token;
            break;
        case fb::ProviderValueKind::Frame: {
            RigExecProviderPlainFrame frame;
            std::copy(row.framePoints.begin(), row.framePoints.end(),
                      frame.points.begin());
            frame.flags = row.frameFlags;
            out.value = frame;
            break;
        }
        case fb::ProviderValueKind::Vec3f:
            out.value = *row.vec3f;
            break;
        case fb::ProviderValueKind::Bool:
            out.value = row.boolean;
            break;
        case fb::ProviderValueKind::Int:
            out.value = int32_t(row.integer);
            break;
        case fb::ProviderValueKind::FloatArray:
            out.value = row.floats;
            break;
        case fb::ProviderValueKind::DoubleArray:
            out.value = row.doubles;
            break;
        case fb::ProviderValueKind::Vec3fArray:
            out.value = row.vec3fs;
            break;
        case fb::ProviderValueKind::Vec3dArray:
            out.value = row.vec3ds;
            break;
        case fb::ProviderValueKind::IntArray:
            out.value = row.ints;
            break;
        case fb::ProviderValueKind::MatrixArray:
            out.value = row.matrices;
            break;
        case fb::ProviderValueKind::TokenArray:
            for (const auto &token : row.tokens)
                internToken(token);
            out.value = row.tokens;
            break;
        case fb::ProviderValueKind::BoolArray: {
            std::vector<bool> bits;
            for (uint8_t bit : row.bools)
                bits.push_back(bit != 0);
            out.value = std::move(bits);
            break;
        }
        case fb::ProviderValueKind::Vec2f:
            out.value = *row.vec2f;
            break;
        case fb::ProviderValueKind::Vec2fArray:
            out.value = row.vec2fs;
            break;
        case fb::ProviderValueKind::Vec3i:
            out.value = *row.vec3i;
            break;
        default:
            if (error)
                *error = "unknown provider value kind";
            return false;
        }
        plain.defaults.push_back(std::move(out));
    }
    for (const auto &row : wire.ops) {
        RigExecProviderPlainOp op;
        op.kind = RigExecProviderOpKind(row.kind);
        op.owner = row.owner;
        op.output = row.output;
        op.inputs = row.inputs;
        op.scaleAvars = row.scaleAvars;
        op.affineKind = row.affineKind;
        op.affineTargets = row.affineTargets;
        plain.ops.push_back(std::move(op));
    }
    const auto leaves = [](const auto &wire, auto *out) {
        for (const auto &row : wire)
            out->push_back({row.value, row.path, row.computation});
    };
    leaves(wire.sampled, &plain.sampled);
    leaves(wire.externalInputs, &plain.externalInputs);
    program->store.providerValues = plain.defaults;
    program->store.providerOpPending.assign(plain.ops.size(), {0, -1});
    program->store.providerAnswerPending.assign(
        RrProviderProducers(program).Count(), {0, 0});
    return true;
}
bool
RrProviderEffectiveInputMemo(const RrProgram *program, int part, int64_t object,
                             std::string *key, std::vector<uint32_t> *covered)
{
    if (part == 2) {
        const auto &leaf =
            program->file->providerProgram->sampled[size_t(object)];
        if (leaf.inputSlot >= 0) {
            RigExecWireInput raw;
            raw.tag = program->file->inputs[size_t(leaf.inputSlot)].type();
            raw.mode = RigExecWireReadMode::Raw;
            raw.walk = {uint32_t(leaf.inputSlot)};
            raw.constant =
                program->file->inputs[size_t(leaf.inputSlot)].value();
            RrSourceReadMemo(program, raw, key);
        }
        return true;
    }
    if (part == 4)
        return true; // Only declared typed frame inputs.
    if (part != 0)
        return false;
    const auto &op = program->providerProgram.ops[size_t(object)];
    const auto &values = program->store.providerValues;
    const auto append = [&](const auto &value) {
        key->append(reinterpret_cast<const char *>(&value), sizeof(value));
    };
    const auto text = [&](const std::string &value) {
        append(uint64_t(value.size()));
        key->append(value);
    };
    const auto state = [&](uint64_t id) {
        const bool exists = id < values.size();
        append(exists);
        if (!exists)
            return;
        const auto &value = values[size_t(id)];
        append(value.initialized);
        append(value.blocked);
        append(value.authoritative);
        append(value.count);
        text(value.error);
        append(uint32_t(value.value.index()));
        std::visit(
            [&](const auto &payload) {
                using T = std::decay_t<decltype(payload)>;
                if constexpr (std::is_same_v<T, std::monostate>) {
                } else if constexpr (std::is_same_v<T, std::string>)
                    text(payload);
                else if constexpr (std::is_same_v<T,
                                                  RigExecProviderPlainFrame>) {
                    for (const auto &point : payload.points)
                        for (double x : point)
                            append(x);
                    append(payload.flags);
                } else if constexpr (std::is_arithmetic_v<T>)
                    append(payload);
                else {
                    append(uint64_t(payload.size()));
                    for (const auto &item : payload) {
                        using E = std::decay_t<decltype(item)>;
                        if constexpr (std::is_same_v<E, std::string>)
                            text(item);
                        else if constexpr (std::is_same_v<T, std::vector<bool>>)
                            append(bool(item));
                        else
                            append(item);
                    }
                }
            },
            value.value);
    };
    const auto input = [&](size_t index) {
        return index < op.inputs.size() ? op.inputs[index] : UINT64_MAX;
    };
    const auto matrix = [&](uint64_t id) -> const std::array<double, 16> * {
        return id < values.size() && values[size_t(id)].initialized &&
                       !values[size_t(id)].blocked
                   ? std::get_if<std::array<double, 16>>(
                         &values[size_t(id)].value)
                   : nullptr;
    };
    const auto frame = [&](uint64_t id) {
        return id < values.size() && values[size_t(id)].initialized &&
               !values[size_t(id)].blocked &&
               std::holds_alternative<RigExecProviderPlainFrame>(
                   values[size_t(id)].value);
    };
    if (op.kind == RigExecProviderOpKind::RestFrame) {
        state(input(7));
        if (input(7) != UINT64_MAX && !frame(input(7)))
            return true;
        state(input(8));
        if (input(8) != UINT64_MAX && !matrix(input(8)))
            return true;
        for (size_t i = 0; i < std::min(size_t(7), op.inputs.size()); ++i)
            state(op.inputs[i]);
        return true;
    }
    if (op.kind == RigExecProviderOpKind::DefaultSpace ||
        op.kind == RigExecProviderOpKind::JointMatrix) {
        const size_t count =
            op.kind == RigExecProviderOpKind::DefaultSpace ? 3 : 2;
        for (size_t i = 0; i < count; ++i) {
            state(input(i));
            if (input(i) != UINT64_MAX &&
                !(i < 2 ? frame(input(i)) : bool(matrix(input(i)))))
                return true;
        }
        for (size_t i = count; i < op.inputs.size(); ++i)
            state(op.inputs[i]);
        return true;
    }
    if (op.kind == RigExecProviderOpKind::Attribute) {
        const uint64_t raw = input(0), connected = input(1);
        const bool authority =
            raw < values.size() && values[size_t(raw)].authoritative;
        append(authority);
        state(authority || connected == UINT64_MAX ? raw : connected);
    } else if (op.kind == RigExecProviderOpKind::SpaceExpression) {
        const uint64_t raw = input(0), connected = input(1),
                       fallback = input(2);
        const auto *authored = matrix(raw);
        const auto *bound = matrix(connected);
        const bool authority =
            raw < values.size() && values[size_t(raw)].authoritative;
        bool identity = true;
        if (authored)
            for (size_t i = 0; i < 16; ++i)
                if ((*authored)[i] != (i % 5 == 0 ? 1.0 : 0.0))
                    identity = false;
        uint8_t selected = authority && authored   ? 0
                           : bound                 ? 1
                           : authored && !identity ? 2
                                                   : 3;
        append(selected);
        state(selected == 0 || selected == 2 ? raw
              : selected == 1                ? connected
                                             : fallback);
    } else if (op.kind == RigExecProviderOpKind::RelativeXform) {
        for (size_t i = 0; i < op.inputs.size(); i += 2) {
            state(op.inputs[i]);
            if (!matrix(op.inputs[i]))
                break;
            if (i + 1 < op.inputs.size()) {
                const uint64_t order = op.inputs[i + 1];
                state(order);
                const auto *tokens =
                    order < values.size() &&
                            values[size_t(order)].initialized &&
                            !values[size_t(order)].blocked
                        ? std::get_if<std::vector<std::string>>(
                              &values[size_t(order)].value)
                        : nullptr;
                if (tokens && std::find(tokens->begin(), tokens->end(),
                                        "!resetXformStack!") != tokens->end())
                    break;
            }
        }
    } else if (op.kind == RigExecProviderOpKind::PosedFrame) {
        const uint64_t posed = input(0);
        const auto *value = matrix(posed);
        bool identity = true;
        if (value)
            for (size_t i = 0; i < 16; ++i)
                if ((*value)[i] != (i % 5 == 0 ? 1.0 : 0.0))
                    identity = false;
        const bool selected =
            value && (values[size_t(posed)].authoritative || !identity);
        append(selected);
        if (selected)
            state(posed);
        else {
            bool missing = false;
            for (size_t i = 1; i <= 3; ++i) {
                state(input(i));
                if (input(i) != UINT64_MAX && !matrix(input(i))) {
                    missing = true;
                    break;
                }
            }
            if (!missing)
                for (size_t i = 4; i < op.inputs.size(); ++i)
                    if (op.scaleAvars || i < 8 || i > 10)
                        state(op.inputs[i]);
        }
    } else if (op.kind == RigExecProviderOpKind::AvarMatrix) {
        for (size_t i = 0; i < op.inputs.size(); ++i)
            if (op.scaleAvars || i < 4 || i > 6)
                state(op.inputs[i]);
    } else {
        for (uint64_t id : op.inputs)
            state(id);
    }
    (void)covered;
    return true;
}
namespace {
bool
PublishSampled(RrProgram *program, size_t row, bool authority, bool blocked)
{
    const auto &leaf = program->file->providerProgram->sampled[row];
    auto &out = program->store.providerValues[size_t(leaf.value)];
    const int slot = leaf.inputSlot;
    out = RigExecProviderPlainState();
    if (slot >= 0 && program->inputState.slotHasValue[size_t(slot)]) {
        const auto &v = program->inputState.slotCurrent[size_t(slot)];
        switch (v.tag) {
        case RigExecWireInputTag::Double: {
            double d;
            std::memcpy(&d, &v.bits, sizeof(d));
            out.value = d;
            break;
        }
        case RigExecWireInputTag::Float: {
            uint32_t bits = uint32_t(v.bits);
            float f;
            std::memcpy(&f, &bits, sizeof(f));
            out.value = f;
            break;
        }
        case RigExecWireInputTag::Matrix4d:
            out.value = v.matrix;
            break;
        case RigExecWireInputTag::Vec3d:
            out.value = v.vec3d;
            break;
        case RigExecWireInputTag::Vec3i:
            out.value = v.vec3i;
            break;
        case RigExecWireInputTag::Vec3f:
            out.value = v.vec3f;
            break;
        case RigExecWireInputTag::Token:
            out.value = program->TextOrEmpty(uint32_t(v.bits));
            break;
        case RigExecWireInputTag::Bool:
            out.value = bool(v.bits);
            break;
        case RigExecWireInputTag::Int:
            out.value = int32_t(uint32_t(v.bits));
            break;
        default:
            break;
        }
        if (RigExecFormatIsArrayTag(v.tag)) {
            const auto copy = [&](const auto *array) {
                if (array)
                    out.value = *array;
            };
            switch (v.tag) {
            case RigExecWireInputTag::IntArray:
                copy(RrInputArray<int32_t>(program, uint32_t(slot)));
                break;
            case RigExecWireInputTag::FloatArray:
                copy(RrInputArray<float>(program, uint32_t(slot)));
                break;
            case RigExecWireInputTag::DoubleArray:
                copy(RrInputArray<double>(program, uint32_t(slot)));
                break;
            case RigExecWireInputTag::Vec3dArray:
                copy(RrInputArray<RigExecWireVec3d>(program, uint32_t(slot)));
                break;
            case RigExecWireInputTag::Matrix4dArray:
                copy(
                    RrInputArray<RigExecWireMatrix4d>(program, uint32_t(slot)));
                break;
            case RigExecWireInputTag::TokenArray: {
                const auto *array =
                    RrInputArray<uint32_t>(program, uint32_t(slot));
                if (array) {
                    std::vector<std::string> text;
                    for (uint32_t id : *array)
                        text.push_back(program->TextOrEmpty(id));
                    out.value = std::move(text);
                }
                break;
            }
            case RigExecWireInputTag::BoolArray: {
                const auto *array =
                    RrInputArray<uint8_t>(program, uint32_t(slot));
                if (array) {
                    std::vector<bool> bits;
                    for (uint8_t bit : *array)
                        bits.push_back(bit != 0);
                    out.value = std::move(bits);
                }
                break;
            }
            case RigExecWireInputTag::Vec3fArray: {
                const auto *array =
                    RrInputArray<RrVec3f>(program, uint32_t(slot));
                if (array) {
                    std::vector<std::array<float, 3>> values;
                    for (const auto &point : *array)
                        values.push_back({point[0], point[1], point[2]});
                    out.value = std::move(values);
                }
                break;
            }
            case RigExecWireInputTag::Vec2fArray: {
                const auto *array =
                    RrInputArray<RrVec2f>(program, uint32_t(slot));
                if (array) {
                    std::vector<std::array<float, 2>> values;
                    for (const auto &point : *array)
                        values.push_back({point[0], point[1]});
                    out.value = std::move(values);
                }
                break;
            }
            default:
                break;
            }
        }
        out.initialized = !std::holds_alternative<std::monostate>(out.value);
        out.count = out.initialized ? 1 : 0;
        if (out.initialized && slot >= 0 &&
            RigExecFormatIsArrayTag(
                program->file->inputs[size_t(slot)].type())) {
            out.count = std::visit(
                [](const auto &value) -> size_t {
                    using T = std::decay_t<decltype(value)>;
                    if constexpr (
                        std::is_same_v<T, std::vector<float>> ||
                        std::is_same_v<T, std::vector<double>> ||
                        std::is_same_v<T, std::vector<std::array<float, 3>>> ||
                        std::is_same_v<T, std::vector<std::array<double, 3>>> ||
                        std::is_same_v<T, std::vector<int32_t>> ||
                        std::is_same_v<T,
                                       std::vector<std::array<double, 16>>> ||
                        std::is_same_v<T, std::vector<std::string>> ||
                        std::is_same_v<T, std::vector<bool>> ||
                        std::is_same_v<T, std::vector<std::array<float, 2>>>)
                        return value.size();
                    return 0;
                },
                out.value);
        }
    }
    // A missing sampled source is a current published monostate, not a
    // never-produced value; native PublishSource also initializes it.
    out.initialized = true;
    out.blocked = blocked;
    out.authoritative = authority;
    return true;
}

bool
PublishRouted(RrProgram *program, size_t step, size_t row, bool authority,
              bool blocked)
{
    const auto &route = program->file->providerProgram->routedInputs[row];
    auto &out = program->store.providerValues[size_t(route.value)];
    RrWireValue value;
    out = RigExecProviderPlainState();
    out.authoritative = authority;
    const auto &cross =
        program->file->crossDomainReads[size_t(route.crossRead)];
    if (cross.kind == fb::CrossDomainReadKind::Points) {
        if (const auto *points = RrReadCrossDomainPoints(program, cross)) {
            std::vector<std::array<float, 3>> current;
            for (const auto &point : *points)
                current.push_back({point[0], point[1], point[2]});
            out.value = std::move(current);
            out.initialized = true;
            out.count = points->size();
        }
    } else if (RrReadCrossDomain(program, route.crossRead, &value)) {
        if (value.tag == RigExecWireInputTag::Matrix4d)
            out.value = value.matrix;
        else if (value.tag == RigExecWireInputTag::Vec3f)
            out.value = value.vec3f;
        else if (value.tag == RigExecWireInputTag::Vec3d)
            out.value = value.vec3d;
        else if (value.tag == RigExecWireInputTag::Vec3i)
            out.value = value.vec3i;
        else if (value.tag == RigExecWireInputTag::Bool)
            out.value = bool(value.bits);
        else if (value.tag == RigExecWireInputTag::Int)
            out.value = int32_t(uint32_t(value.bits));
        else if (value.tag == RigExecWireInputTag::Token)
            out.value = program->TextOrEmpty(uint32_t(value.bits));
        else if (value.tag == RigExecWireInputTag::Double) {
            double d;
            std::memcpy(&d, &value.bits, sizeof(d));
            out.value = d;
        } else if (value.tag == RigExecWireInputTag::Float) {
            uint32_t bits = uint32_t(value.bits);
            float f;
            std::memcpy(&f, &bits, sizeof(f));
            out.value = f;
        }
        out.initialized = !std::holds_alternative<std::monostate>(out.value);
        out.count = out.initialized ? 1 : 0;
    }
    out.blocked = blocked;
    out.initialized = true;
    if (blocked && !cross.unavailable.empty() &&
        step < program->store.stepOutputs.size())
        program->store.stepOutputs[step].diagnostics.push_back(
            cross.unavailable);
    return true;
}

bool
PublishExternal(RrProgram *program, size_t row)
{
    const auto &leaf = program->file->providerProgram->externalInputs[row];
    Store store(program->store.providerValues);
    const int slot = leaf.providerSlot;
    const auto *pose = static_cast<const RrPoseScratch *>(program->pose.get());
    if (leaf.computation == SchemaComputationText::computeRestFrame)
        store.PublishFrame(leaf.value, slot >= 0
                                           ? pose->restFrames[size_t(slot)]
                                           : RrFrameFromMatrix(RrMat4d(1)));
    else if (leaf.computation == SchemaComputationText::computePointFrame ||
             leaf.computation == SchemaComputationText::computeBasePointFrame) {
        // Initial provider composition reads its initial base frame.
        // Contextual refreshed versions are explicit part4 bridges.
        store.PublishFrame(leaf.value, slot >= 0
                                           ? program->store.base[size_t(slot)]
                                           : RrFrameFromMatrix(RrMat4d(1)));
    } else {
        RrMat4d matrix(1);
        if (leaf.interveningRead) {
            const auto value = RrReadInput(program, *leaf.interveningRead);
            std::memcpy(matrix._mtx, value.matrix.data(), sizeof(double) * 16);
        }
        store.PublishMatrix(leaf.value, matrix, false);
    }
    return true;
}

bool
PublishPoseFrame(RrProgram *program, size_t row)
{
    const auto &leaf = program->poses->providerFrameInputs[row];
    Store bridge(program->store.providerValues);
    bridge.PublishFrame(leaf.value, leaf.base
                                        ? program->store.base[leaf.version]
                                        : program->store.fin[leaf.version]);
    return true;
}

/// Whether routed input \p row's cross-domain read answers a value
/// PublishRouted keeps.
bool
RoutedAvailable(const RrProgram *program, size_t row)
{
    const auto &route = program->file->providerProgram->routedInputs[row];
    const auto &cross =
        program->file->crossDomainReads[size_t(route.crossRead)];
    if (cross.kind == fb::CrossDomainReadKind::Points)
        return RrReadCrossDomainPoints(program, cross) != nullptr;
    RrWireValue value;
    if (!RrReadCrossDomain(program, route.crossRead, &value))
        return false;
    switch (value.tag) {
    case RigExecWireInputTag::Matrix4d:
    case RigExecWireInputTag::Vec3f:
    case RigExecWireInputTag::Vec3d:
    case RigExecWireInputTag::Vec3i:
    case RigExecWireInputTag::Bool:
    case RigExecWireInputTag::Int:
    case RigExecWireInputTag::Token:
    case RigExecWireInputTag::Double:
    case RigExecWireInputTag::Float:
        return true;
    default:
        return false;
    }
}
} // namespace

eval::spaces::ProviderProducers
RrProviderProducers(const RrProgram *program)
{
    eval::spaces::ProviderProducers producers;
    producers.operations = program->providerProgram.ops.size();
    if (program->file && program->file->providerProgram) {
        const auto &wire = *program->file->providerProgram;
        producers.sampled = wire.sampled.size();
        producers.external = wire.externalInputs.size();
        producers.routed = wire.routedInputs.size();
    }
    if (program->poses)
        producers.frames = program->poses->providerFrameInputs.size();
    return producers;
}

bool
RrProviderCandidates(const RrProgram *program, const eval::OpLookup &lookup,
                     const RigExecTypeDesc &type, void *out, std::string *error)
{
    using Producer = eval::spaces::ProviderProducer;
    using Read = eval::spaces::ProviderCandidateRead;
    Producer part = Producer::Operation;
    size_t row = 0;
    if (!type.element || lookup.read > uint8_t(Read::Blocked) ||
        !RrProviderProducers(program).Of(lookup.object, &part, &row)) {
        if (error)
            *error = "provider value producer " +
                     std::to_string(lookup.object) + " has no candidates";
        return false;
    }
    const Read read = Read(lookup.read);
    auto &list = *static_cast<rigExecStd::RigExecArray<uint8_t> *>(out);
    const auto flag = [&](bool present, bool authoritative, bool blocked) {
        return uint8_t(read == Read::Present         ? present
                       : read == Read::Authoritative ? authoritative
                                                     : blocked);
    };
    if (part == Producer::Operation) {
        const auto &op = program->providerProgram.ops[row];
        const auto &values = program->store.providerValues;
        const uint64_t raw = op.inputs.empty() ? UINT64_MAX : op.inputs[0];
        const bool connected =
            op.inputs.size() > 1 && op.inputs[1] != UINT64_MAX;
        list.Assign(connected ? 2 : 1);
        for (size_t k = 0; k < list.Size(); ++k) {
            const uint64_t id = k ? op.inputs[1] : raw;
            const RigExecProviderPlainState *state =
                id < values.size() ? &values[size_t(id)] : nullptr;
            list[k] =
                flag(state && state->initialized, state && state->authoritative,
                     state && state->blocked);
        }
        return true;
    }
    list.Assign(1);
    if (part == Producer::Sampled) {
        const int slot = program->file->providerProgram->sampled[row].inputSlot;
        list[0] = flag(
            true, slot >= 0 && program->inputState.slotAuthored[size_t(slot)],
            slot >= 0 && program->inputState.slotBlocked[size_t(slot)] != 0);
    } else if (part == Producer::Routed) {
        list[0] = flag(true, true, !RoutedAvailable(program, row));
    } else {
        list[0] = flag(true, false, false);
    }
    return true;
}

bool
RrStoreProviderAnswer(RrProgram *program, size_t step,
                      const eval::OpHostOutput &output, const void *value,
                      std::string *error)
{
    using Producer = eval::spaces::ProviderProducer;
    using T = eval::OpOutputTarget;
    RrStore &store = program->store;
    Producer part = Producer::Operation;
    size_t row = 0;
    if (output.object >= store.providerAnswerPending.size() ||
        !RrProviderProducers(program).Of(output.object, &part, &row)) {
        if (error)
            *error =
                "no provider value producer " + std::to_string(output.object);
        return false;
    }
    auto &pending = store.providerAnswerPending[output.object];
    if (output.target == uint8_t(T::ProviderAnswerAuthority)) {
        pending.first = *static_cast<const uint8_t *>(value);
        return true;
    }
    if (output.target == uint8_t(T::ProviderAnswerBlocked)) {
        pending.second = *static_cast<const uint8_t *>(value);
        return true;
    }
    const bool authority = pending.first != 0, blocked = pending.second != 0;
    pending = {0, 0};
    const int32_t selected = *static_cast<const int32_t *>(value);
    auto &values = store.providerValues;
    uint64_t answered = UINT64_MAX;
    switch (part) {
    case Producer::Operation:
        answered = program->providerProgram.ops[row].output;
        break;
    case Producer::Sampled:
        answered = program->file->providerProgram->sampled[row].value;
        break;
    case Producer::External:
        answered = program->file->providerProgram->externalInputs[row].value;
        break;
    case Producer::Routed:
        answered = program->file->providerProgram->routedInputs[row].value;
        break;
    case Producer::PoseFrame:
        answered = program->poses->providerFrameInputs[row].value;
        break;
    }
    if (answered >= values.size()) {
        if (error)
            *error = "provider output is outside its typed layout";
        return false;
    }
    if (selected < 0) {
        auto &out = values[size_t(answered)];
        out.value = std::monostate{};
        out.initialized = true;
        out.blocked = blocked;
        out.authoritative = false;
        out.count = 0;
        out.error = "provider input unavailable";
        return true;
    }
    switch (part) {
    case Producer::Operation: {
        const auto &op = program->providerProgram.ops[row];
        if (size_t(selected) >= op.inputs.size() ||
            op.inputs[size_t(selected)] >= values.size()) {
            if (error)
                *error = "provider operation " + std::to_string(row) +
                         " has no candidate " + std::to_string(selected);
            return false;
        }
        values[size_t(answered)] = values[size_t(op.inputs[size_t(selected)])];
        values[size_t(answered)].authoritative = authority;
        return true;
    }
    case Producer::Sampled:
        return PublishSampled(program, row, authority, blocked);
    case Producer::Routed:
        return PublishRouted(program, step, row, authority, blocked);
    case Producer::External:
        return PublishExternal(program, row);
    case Producer::PoseFrame:
        return PublishPoseFrame(program, row);
    }
    return false;
}

namespace {

using ProviderRead = eval::spaces::ProviderRead;
using ProviderClass = eval::spaces::ProviderInputClass;
using AffineClass = eval::spaces::AffineInputClass;

ProviderClass
ArithmeticClassOf(const RigExecProviderPlainOp &op, size_t p)
{
    return op.inputs[p] == UINT64_MAX
               ? ProviderClass::None
               : eval::spaces::ProviderArithmeticInputClass(op.kind, p);
}

AffineClass
AffineClassOf(const RigExecProviderPlainOp &op, size_t p)
{
    return eval::spaces::AffineFrameInputClass<RrAffineMath>(op.affineKind, p);
}

bool
ItemsClass(AffineClass c)
{
    return c == AffineClass::IntItems || c == AffineClass::DoubleItems ||
           c == AffineClass::MatrixItems;
}

template <class T>
rigExecStd::RigExecArray<T> &
ListOf(void *out)
{
    return *static_cast<rigExecStd::RigExecArray<T> *>(out);
}

/// \p id's value as a T when it is present and unblocked, as Store reads.
template <class T>
const T *
ReadAs(const std::vector<RigExecProviderPlainState> &values, uint64_t id)
{
    return id < values.size() && values[size_t(id)].initialized &&
                   !values[size_t(id)].blocked
               ? std::get_if<T>(&values[size_t(id)].value)
               : nullptr;
}

bool
ReadScalar(const std::vector<RigExecProviderPlainState> &values, uint64_t id,
           double *out)
{
    if (const auto *value = ReadAs<double>(values, id)) {
        *out = *value;
        return true;
    }
    if (const auto *narrow = ReadAs<float>(values, id)) {
        *out = double(*narrow);
        return true;
    }
    return false;
}

void
PutMatrix(const std::array<double, 16> *matrix, rigExecStd::Mat4d *out)
{
    static_assert(sizeof(rigExecStd::Mat4d) == 16 * sizeof(double),
                  "a std matrix is sixteen doubles");
    const std::array<double, 16> identity = {1, 0, 0, 0, 0, 1, 0, 0,
                                             0, 0, 1, 0, 0, 0, 0, 1};
    std::memcpy(static_cast<void *>(out), (matrix ? matrix : &identity)->data(),
                16 * sizeof(double));
}

void
PutFrame(const RigExecProviderPlainFrame &frame, rigExecRig::PointFrame *out)
{
    for (size_t p = 0; p < 4; ++p)
        out->points[p] = rigExecStd::Vec3d(
            frame.points[p][0], frame.points[p][1], frame.points[p][2]);
    out->flags = frame.flags;
}

bool
ArithmeticReadable(const std::vector<RigExecProviderPlainState> &values,
                   uint64_t id, ProviderClass c)
{
    double scalar = 0.0;
    switch (c) {
    case ProviderClass::Matrix:
        return ReadAs<std::array<double, 16>>(values, id) != nullptr;
    case ProviderClass::Frame:
        return ReadAs<RigExecProviderPlainFrame>(values, id) != nullptr;
    case ProviderClass::Scalar:
        return ReadScalar(values, id, &scalar);
    case ProviderClass::Vector:
        return ReadAs<std::array<double, 3>>(values, id) != nullptr;
    case ProviderClass::Order:
    case ProviderClass::Reset:
    case ProviderClass::None:
        break;
    }
    return true;
}

bool
AffineReadable(const std::vector<RigExecProviderPlainState> &values,
               uint64_t id, AffineClass c)
{
    double scalar = 0.0;
    switch (c) {
    case AffineClass::Matrix:
        return ReadAs<std::array<double, 16>>(values, id) != nullptr;
    case AffineClass::Vector:
        return ReadAs<std::array<double, 3>>(values, id) != nullptr;
    case AffineClass::IntVector:
        return ReadAs<std::array<int32_t, 3>>(values, id) != nullptr;
    case AffineClass::Double:
        return ReadScalar(values, id, &scalar);
    case AffineClass::Bool:
        return ReadAs<bool>(values, id) != nullptr;
    case AffineClass::Int:
        return ReadAs<int32_t>(values, id) != nullptr;
    case AffineClass::Frame:
        return ReadAs<RigExecProviderPlainFrame>(values, id) != nullptr;
    case AffineClass::Text:
    case AffineClass::IntItems:
    case AffineClass::DoubleItems:
    case AffineClass::MatrixItems:
        break;
    }
    return true;
}

bool
ResetsStack(const std::vector<RigExecProviderPlainState> &values, uint64_t id)
{
    const auto *tokens = ReadAs<std::vector<std::string>>(values, id);
    return tokens && std::find(tokens->begin(), tokens->end(),
                               "!resetXformStack!") != tokens->end();
}

bool
FillArithmetic(const std::vector<RigExecProviderPlainState> &values,
               const RigExecProviderPlainOp &op, ProviderRead read, void *out,
               std::string *error)
{
    const size_t n = op.inputs.size();
    const auto count = [&](ProviderClass c) {
        size_t k = 0;
        for (size_t p = 0; p < n; ++p)
            k += ArithmeticClassOf(op, p) == c ? 1 : 0;
        return k;
    };
    const auto each = [&](ProviderClass c, auto &&put) {
        size_t k = 0;
        for (size_t p = 0; p < n; ++p)
            if (ArithmeticClassOf(op, p) == c)
                put(k++, op.inputs[p]);
    };
    switch (read) {
    case ProviderRead::Present: {
        auto &list = ListOf<uint8_t>(out);
        list.Assign(n);
        for (size_t p = 0; p < n; ++p)
            list[p] = ArithmeticReadable(values, op.inputs[p],
                                         ArithmeticClassOf(op, p))
                          ? 1
                          : 0;
        return true;
    }
    case ProviderRead::Matrices: {
        auto &list = ListOf<rigExecStd::Mat4d>(out);
        list.Assign(count(ProviderClass::Matrix));
        each(ProviderClass::Matrix, [&](size_t k, uint64_t id) {
            PutMatrix(ReadAs<std::array<double, 16>>(values, id), &list[k]);
        });
        return true;
    }
    case ProviderRead::Authoritative: {
        auto &list = ListOf<uint8_t>(out);
        list.Assign(count(ProviderClass::Matrix));
        each(ProviderClass::Matrix, [&](size_t k, uint64_t id) {
            list[k] =
                id < values.size() && values[size_t(id)].authoritative ? 1 : 0;
        });
        return true;
    }
    case ProviderRead::Frames: {
        auto &list = ListOf<rigExecRig::PointFrame>(out);
        list.Assign(count(ProviderClass::Frame));
        each(ProviderClass::Frame, [&](size_t k, uint64_t id) {
            if (const auto *frame =
                    ReadAs<RigExecProviderPlainFrame>(values, id))
                PutFrame(*frame, &list[k]);
        });
        return true;
    }
    case ProviderRead::Scalars: {
        auto &list = ListOf<double>(out);
        list.Assign(count(ProviderClass::Scalar));
        each(ProviderClass::Scalar,
             [&](size_t k, uint64_t id) { ReadScalar(values, id, &list[k]); });
        return true;
    }
    case ProviderRead::Vectors: {
        auto &list = ListOf<rigExecStd::Vec3d>(out);
        list.Assign(count(ProviderClass::Vector));
        each(ProviderClass::Vector, [&](size_t k, uint64_t id) {
            if (const auto *v = ReadAs<std::array<double, 3>>(values, id))
                list[k] = rigExecStd::Vec3d((*v)[0], (*v)[1], (*v)[2]);
        });
        return true;
    }
    case ProviderRead::Orders: {
        auto &list = ListOf<int32_t>(out);
        list.Assign(count(ProviderClass::Order));
        each(ProviderClass::Order, [&](size_t k, uint64_t id) {
            if (const auto *token = ReadAs<std::string>(values, id))
                list[k] = eval::spaces::ProviderPackOrder(*token);
        });
        return true;
    }
    case ProviderRead::Resets: {
        auto &list = ListOf<uint8_t>(out);
        list.Assign(count(ProviderClass::Reset));
        each(ProviderClass::Reset, [&](size_t k, uint64_t id) {
            list[k] = ResetsStack(values, id) ? 1 : 0;
        });
        return true;
    }
    default:
        break;
    }
    if (error)
        *error = "provider arithmetic has no input list " +
                 std::to_string(int(read));
    return false;
}

bool
FillAffine(const std::vector<RigExecProviderPlainState> &values,
           const RigExecProviderPlainOp &op, ProviderRead read, void *out,
           std::string *error)
{
    const size_t n = op.inputs.size();
    const auto listed = [&](size_t p, AffineClass c) {
        return op.inputs[p] != UINT64_MAX && AffineClassOf(op, p) == c;
    };
    const auto fill = [&](AffineClass c, auto &list, auto &&put) {
        size_t k = 0;
        for (size_t p = 0; p < n; ++p)
            k += listed(p, c) ? 1 : 0;
        list.Assign(k);
        k = 0;
        for (size_t p = 0; p < n; ++p)
            if (listed(p, c))
                put(k++, op.inputs[p]);
    };
    const auto items = [&](AffineClass c, auto &list, const auto *sample) {
        using Array = std::decay_t<decltype(*sample)>;
        size_t total = 0;
        for (size_t p = 0; p < n; ++p)
            if (listed(p, c))
                if (const auto *array = ReadAs<Array>(values, op.inputs[p]))
                    total += array->size();
        list.Assign(total);
        size_t at = 0;
        for (size_t p = 0; p < n; ++p)
            if (listed(p, c))
                if (const auto *array = ReadAs<Array>(values, op.inputs[p]))
                    for (const auto &item : *array)
                        std::memcpy(static_cast<void *>(&list[at++]), &item,
                                    sizeof(item));
    };
    const auto size = [&](AffineClass c, uint64_t id) -> size_t {
        if (c == AffineClass::IntItems) {
            const auto *array = ReadAs<std::vector<int32_t>>(values, id);
            return array ? array->size() : 0;
        }
        if (c == AffineClass::DoubleItems) {
            const auto *array = ReadAs<std::vector<double>>(values, id);
            return array ? array->size() : 0;
        }
        const auto *array =
            ReadAs<std::vector<std::array<double, 16>>>(values, id);
        return array ? array->size() : 0;
    };
    switch (read) {
    case ProviderRead::AffinePresent: {
        auto &list = ListOf<uint8_t>(out);
        list.Assign(n);
        for (size_t p = 0; p < n; ++p)
            list[p] = op.inputs[p] == UINT64_MAX ||
                              AffineReadable(values, op.inputs[p],
                                             AffineClassOf(op, p))
                          ? 1
                          : 0;
        return true;
    }
    case ProviderRead::AffineMatrices: {
        auto &list = ListOf<rigExecStd::Mat4d>(out);
        fill(AffineClass::Matrix, list, [&](size_t k, uint64_t id) {
            PutMatrix(ReadAs<std::array<double, 16>>(values, id), &list[k]);
        });
        return true;
    }
    case ProviderRead::AffineDoubles: {
        auto &list = ListOf<double>(out);
        fill(AffineClass::Double, list,
             [&](size_t k, uint64_t id) { ReadScalar(values, id, &list[k]); });
        return true;
    }
    case ProviderRead::AffineBools: {
        auto &list = ListOf<uint8_t>(out);
        fill(AffineClass::Bool, list, [&](size_t k, uint64_t id) {
            const bool *value = ReadAs<bool>(values, id);
            list[k] = value && *value ? 1 : 0;
        });
        return true;
    }
    case ProviderRead::AffineInts: {
        auto &list = ListOf<int32_t>(out);
        fill(AffineClass::Int, list, [&](size_t k, uint64_t id) {
            const int32_t *value = ReadAs<int32_t>(values, id);
            list[k] = value ? *value : 0;
        });
        return true;
    }
    case ProviderRead::AffineVectors: {
        auto &list = ListOf<rigExecStd::Vec3d>(out);
        fill(AffineClass::Vector, list, [&](size_t k, uint64_t id) {
            if (const auto *v = ReadAs<std::array<double, 3>>(values, id))
                list[k] = rigExecStd::Vec3d((*v)[0], (*v)[1], (*v)[2]);
        });
        return true;
    }
    case ProviderRead::AffineIntVectors: {
        auto &list = ListOf<rigExecStd::Vec3i>(out);
        fill(AffineClass::IntVector, list, [&](size_t k, uint64_t id) {
            if (const auto *v = ReadAs<std::array<int32_t, 3>>(values, id))
                list[k] = rigExecStd::Vec3i((*v)[0], (*v)[1], (*v)[2]);
        });
        return true;
    }
    case ProviderRead::AffineFrames: {
        auto &list = ListOf<rigExecRig::PointFrame>(out);
        fill(AffineClass::Frame, list, [&](size_t k, uint64_t id) {
            if (const auto *frame =
                    ReadAs<RigExecProviderPlainFrame>(values, id))
                PutFrame(*frame, &list[k]);
        });
        return true;
    }
    case ProviderRead::AffineIntItems:
        items(AffineClass::IntItems, ListOf<int32_t>(out),
              static_cast<const std::vector<int32_t> *>(nullptr));
        return true;
    case ProviderRead::AffineDoubleItems:
        items(AffineClass::DoubleItems, ListOf<double>(out),
              static_cast<const std::vector<double> *>(nullptr));
        return true;
    case ProviderRead::AffineMatrixItems:
        items(
            AffineClass::MatrixItems, ListOf<rigExecStd::Mat4d>(out),
            static_cast<const std::vector<std::array<double, 16>> *>(nullptr));
        return true;
    case ProviderRead::AffineItemSpans: {
        size_t spans = 0;
        for (size_t p = 0; p < n; ++p)
            spans +=
                op.inputs[p] != UINT64_MAX && ItemsClass(AffineClassOf(op, p))
                    ? 1
                    : 0;
        auto &list = ListOf<int32_t>(out);
        list.Assign(2 * spans);
        size_t k = 0, offsets[3] = {0, 0, 0};
        for (size_t p = 0; p < n; ++p) {
            const AffineClass c = AffineClassOf(op, p);
            if (op.inputs[p] == UINT64_MAX || !ItemsClass(c))
                continue;
            size_t &offset = offsets[c == AffineClass::IntItems      ? 0
                                     : c == AffineClass::DoubleItems ? 1
                                                                     : 2];
            const size_t count = size(c, op.inputs[p]);
            list[2 * k] = int32_t(offset);
            list[2 * k + 1] = int32_t(count);
            offset += count;
            ++k;
        }
        return true;
    }
    default:
        break;
    }
    if (error)
        *error =
            "an affine frame has no input list " + std::to_string(int(read));
    return false;
}

} // namespace

bool
RrFillProviderInputs(const RrProgram *program, const eval::OpLookup &lookup,
                     const RigExecTypeDesc &type, void *out, std::string *error)
{
    const auto &ops = program->providerProgram.ops;
    if (lookup.object >= ops.size() || !type.element) {
        if (error)
            *error = "no provider operation " + std::to_string(lookup.object) +
                     " fills a list";
        return false;
    }
    const RigExecProviderPlainOp &op = ops[lookup.object];
    const ProviderRead read = ProviderRead(lookup.read);
    return op.kind == RigExecProviderOpKind::AffineFrame
               ? FillAffine(program->store.providerValues, op, read, out, error)
               : FillArithmetic(program->store.providerValues, op, read, out,
                                error);
}

bool
RrStoreProviderValue(RrProgram *program, const eval::OpHostOutput &output,
                     const void *value, std::string *error)
{
    using T = eval::OpOutputTarget;
    const auto &ops = program->providerProgram.ops;
    RrStore &store = program->store;
    if (output.object >= ops.size() ||
        output.object >= store.providerOpPending.size()) {
        if (error)
            *error = "no provider operation " + std::to_string(output.object);
        return false;
    }
    auto &pending = store.providerOpPending[output.object];
    const T target = T(output.target);
    if (target == T::ProviderValueAuthority) {
        pending.first = *static_cast<const uint8_t *>(value);
        return true;
    }
    if (target == T::ProviderValueUnavailable) {
        pending.second = *static_cast<const int32_t *>(value);
        return true;
    }
    const RigExecProviderPlainOp &op = ops[output.object];
    const auto [authority, unavailable] = pending;
    pending = {0, -1};
    auto &values = store.providerValues;
    if (op.output >= values.size()) {
        if (error)
            *error = "provider output is outside its typed layout";
        return false;
    }
    Store state(values);
    if (unavailable >= 0) {
        state.Invalidate(op.output, size_t(unavailable) < op.inputs.size()
                                        ? op.inputs[size_t(unavailable)]
                                        : UINT64_MAX);
        return true;
    }
    if (target == T::ProviderValueFrame) {
        RrPointFrame frame;
        static_assert(sizeof(rigExecRig::PointFrame) == sizeof(RrPointFrame),
                      "a rig frame is a runtime frame");
        std::memcpy(static_cast<void *>(&frame), value, sizeof(RrPointFrame));
        state.PublishFrame(op.output, frame);
        return true;
    }
    state.PublishMatrix(op.output, *static_cast<const RrMat4d *>(value),
                        authority != 0);
    return true;
}

} // namespace rigExec
