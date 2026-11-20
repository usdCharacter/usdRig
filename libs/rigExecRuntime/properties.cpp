// Bind playback property parts to decoded reads and typed result slots.
#include "rigExecRuntime/store.h"
#include "rigExecEvalCore/properties.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>

namespace rigExec {

namespace {

using _RrTag = RrPropertyValue::Tag;

// What Open settled about the chains: the diagnostic texts, the parsed
// operations, the curve keys as the kernel reads them, each chain's range
// of phased consumers, and where each result is published.
struct RrPropertyScratch {
    struct Revision {
        std::string mover; ///< prim path text, for diagnostics
        bool opValid = false;
        RigExecPropertyOp op = RigExecPropertyOp::Add;
        std::vector<RrVec2f> keys;
        std::vector<RrVec2f> tangents;
        /// RigExecValidateLinearKeys over the keys (static data).
        bool keysValid = false;
        /// A path revision's settings and its curve points' array slot
        /// (-1: none, which passes the value through).
        RigExecPathSettings path;
        int32_t pathPoints = -1;
    };
    struct Chain {
        std::string target; ///< target path text, for diagnostics
        /// The slot type the chain's typed base read needs.
        RigExecWireInputTag baseTag = RigExecWireInputTag::Float;
        /// propertyValues entry of the target.
        size_t publish = 0;
        /// [phasedBegin, phasedEnd) in the file's phasedConsumers.
        size_t phasedBegin = 0;
        size_t phasedEnd = 0;
        std::vector<Revision> revisions;
    };
    std::vector<Chain> chains;
    /// propertyValues entry per phased consumer.
    std::vector<size_t> phasedPublish;
    /// Per propertyValues entry: the attribute path id it is published at.
    std::vector<uint32_t> publishNames;
};

bool
_RrFail(std::string *error, const std::string &why)
{
    if (error) {
        *error = why;
    }
    return false;
}

// Every member set, so two values compare equal exactly when their tagged
// member does (RrPropertyValue::operator== compares all of them).
RrPropertyValue
_RrPropertyZero()
{
    RrPropertyValue value;
    value.tag = _RrTag::Float;
    value.f32 = 0.0f;
    value.f64 = 0.0;
    value.matrix.SetDiagonal(0.0);
    value.vec = RrVec3f(0.0f);
    return value;
}

RrPropertyValue
_RrHold(float v)
{
    RrPropertyValue value = _RrPropertyZero();
    value.f32 = v;
    return value;
}

RrPropertyValue
_RrHold(double v)
{
    RrPropertyValue value = _RrPropertyZero();
    value.tag = _RrTag::Double;
    value.f64 = v;
    return value;
}

RrPropertyValue
_RrHold(const RrMat4d &m)
{
    RrPropertyValue value = _RrPropertyZero();
    value.tag = _RrTag::Matrix4d;
    value.matrix = m;
    return value;
}

RrPropertyValue
_RrHold(const RrVec3f &v)
{
    RrPropertyValue value = _RrPropertyZero();
    value.tag = _RrTag::Vec3f;
    value.vec = v;
    return value;
}

RrMat4d
_RrMatrix(const RigExecWireMatrix4d &m)
{
    RrMat4d out;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            out[r][c] = m[r * 4 + c];
        }
    }
    return out;
}

RrVec3f
_RrVec3f(const RrWireValue &value)
{
    return RrVec3f(value.vec3f[0], value.vec3f[1], value.vec3f[2]);
}

void
_RrPublish(RrProgram *program, const RrPropertyScratch &scratch, size_t entry,
           const RrPropertyValue &value)
{
    RrStore &store = program->store;
    store.propertyValues[entry] = value;
    store.propertyPublished[entry] = 1;
    store.propertyResults[scratch.publishNames[entry]] = value;
}

struct RrPropertyHost {
    struct Math {
        using Matrix = RrMat4d;
        using Vec3 = RrVec3f;
        using Key = RrVec2f;
    };

    struct Revision {
        const RrProgram &program;
        const RigExecWirePropertyRevision &wire;
        const RrPropertyScratch::Revision &bound;

        bool exists() const
        {
            return true;
        }
        std::string label() const
        {
            return "diag " + bound.mover;
        }
        void appendInputDiagnostics(std::vector<std::string> &) const
        {
        }
        bool enabled() const
        {
            return RrReadInput(&program, *wire.enabled).bits != 0;
        }
        bool hasWeightField() const
        {
            return wire.envelope >= 0;
        }
        eval::properties::WeightFieldView weightField() const
        {
            const auto &field =
                program.store.weightFieldResults[size_t(wire.weightField)];
            return {field.ok, field.values.size(), field.values.data(),
                    field.error};
        }
        float defaultWeight() const
        {
            return RrWireValueFloat(RrReadInput(&program, *wire.defaultWeight));
        }
        bool opValid() const
        {
            return bound.opValid;
        }
        RigExecPropertyOp op() const
        {
            return bound.op;
        }
        template <class T> T read(eval::properties::Operand operand) const
        {
            using O = eval::properties::Operand;
            const auto &input = operand == O::Value     ? wire.value
                                : operand == O::Minimum ? wire.min
                                                        : wire.max;
            const auto value = RrReadInput(&program, *input);
            if constexpr (std::is_same_v<T, float>)
                return RrWireValueFloat(value);
            else if constexpr (std::is_same_v<T, RrMat4d>)
                return _RrMatrix(value.matrix);
            else
                return _RrVec3f(value);
        }
        eval::properties::ArrayView<RrVec2f> keys() const
        {
            return {bound.keys.data(), bound.keys.size()};
        }
        eval::properties::ArrayView<RrVec2f> tangents() const
        {
            return {bound.tangents.data(), bound.tangents.size()};
        }
        bool keysValid(eval::properties::ArrayView<RrVec2f>) const
        {
            return bound.keysValid;
        }
        eval::properties::ArrayView<RrVec3f> pathPoints() const
        {
            const auto *points = bound.pathPoints >= 0
                                     ? RrInputArray<RrVec3f>(
                                           &program, uint32_t(bound.pathPoints))
                                     : nullptr;
            return points ? eval::properties::ArrayView<RrVec3f>{points->data(),
                                                                 points->size()}
                          : eval::properties::ArrayView<RrVec3f>{};
        }
        const RigExecPathSettings &pathSettings() const
        {
            return bound.path;
        }
    };

    struct Inputs {
        const RrProgram &program;
        const RigExecWireFile &file;
        const RrPropertyScratch &scratch;

        size_t chainCount() const
        {
            return file.propertyChains.size();
        }
        size_t revisionCount(size_t c) const
        {
            return file.propertyChains[c].revisions.size();
        }
        size_t versionBase(size_t c) const
        {
            return file.propertyChains[c].versionBase;
        }
        auto valueType(size_t c) const
        {
            return file.propertyChains[c].valueType;
        }
        std::string baseLabel(size_t c) const
        {
            return "property chain " + scratch.chains[c].target;
        }
        template <class T>
        eval::properties::BaseRead readBase(size_t c, T &value) const
        {
            using B = eval::properties::BaseRead;
            RrWireValue base;
            if (!RrChainBase(&program, c, scratch.chains[c].baseTag, &base))
                return B::Missing;
            if constexpr (std::is_same_v<T, float>)
                value = RrWireValueFloat(base);
            else if constexpr (std::is_same_v<T, double>)
                std::memcpy(&value, &base.bits, sizeof(value));
            else if constexpr (std::is_same_v<T, RrMat4d>)
                value = _RrMatrix(base.matrix);
            else
                value = _RrVec3f(base);
            return B::Present;
        }
        Revision revision(size_t c, size_t r) const
        {
            return {program, file.propertyChains[c].revisions[r],
                    scratch.chains[c].revisions[r]};
        }
        template <class Fn> void forEachRecord(size_t c, Fn fn) const
        {
            const auto &chain = scratch.chains[c];
            for (size_t k = chain.phasedBegin; k < chain.phasedEnd; ++k)
                fn(k, file.phasedConsumers[k]);
        }
    };

    struct State {
        RrStore &store;
        std::vector<std::string> &lines;
        void setChainValid(size_t c, bool valid)
        {
            store.propertyChainValid[c] = valid;
        }
        bool predecessorValid(size_t c, size_t) const
        {
            return store.propertyChainValid[c];
        }
        template <class T> const T &version(size_t id) const
        {
            const auto &value = store.propertyVersions[id];
            if constexpr (std::is_same_v<T, float>)
                return value.f32;
            else if constexpr (std::is_same_v<T, double>)
                return value.f64;
            else if constexpr (std::is_same_v<T, RrMat4d>)
                return value.matrix;
            else
                return value.vec;
        }
        template <class T>
        void storeVersion(size_t id, const T *value, bool valid)
        {
            if (value)
                store.propertyVersions[id] = _RrHold(*value);
            store.propertyVersionValid[id] = valid;
        }
        bool recordStoodAside(size_t k) const
        {
            return store.propertyRecordStoodAside[k];
        }
    };

    Inputs input;
    State output;
    const Inputs &inputs() const
    {
        return input;
    }
    State &state()
    {
        return output;
    }
    const RigExecRangeRunner *ranges() const
    {
        return nullptr;
    }
};

} // namespace

bool
RrPropertySizeScratch(RrProgram *program, std::string *error)
{
    auto scratch = std::make_shared<RrPropertyScratch>();
    RrStore &store = program->store;
    const RigExecWireFile *file = program->inputState.file;
    const size_t slots = file->inputs.size();
    std::vector<int64_t> slotEntry(slots, -1);
    const auto entryOf = [&](uint32_t slot) {
        if (slotEntry[slot] < 0) {
            slotEntry[slot] = int64_t(scratch->publishNames.size());
            scratch->publishNames.push_back(file->inputs[slot].name());
        }
        return size_t(slotEntry[slot]);
    };
    scratch->chains.resize(file->propertyChains.size());
    for (size_t c = 0; c < file->propertyChains.size(); ++c) {
        const RigExecWirePropertyChain &chain = file->propertyChains[c];
        RrPropertyScratch::Chain &plan = scratch->chains[c];
        if (chain.target >= slots) {
            return _RrFail(error, "a property chain targets no input "
                                  "slot");
        }
        plan.target = program->TextOrEmpty(file->inputs[chain.target].name());
        switch (chain.valueType) {
        case RigExecWirePropertyValueType::Float:
            plan.baseTag = RigExecWireInputTag::Float;
            break;
        case RigExecWirePropertyValueType::Double:
            plan.baseTag = RigExecWireInputTag::Double;
            break;
        case RigExecWirePropertyValueType::Matrix4d:
            plan.baseTag = RigExecWireInputTag::Matrix4d;
            break;
        case RigExecWirePropertyValueType::Vec3f:
            plan.baseTag = RigExecWireInputTag::Vec3f;
            break;
        default:
            return _RrFail(error, "property chain " + plan.target +
                                      " has an unknown value type");
        }
        plan.publish = entryOf(chain.target);
        plan.revisions.resize(chain.revisions.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecWirePropertyRevision &revision = chain.revisions[r];
            RrPropertyScratch::Revision &bound = plan.revisions[r];
            bound.mover = program->TextOrEmpty(revision.mover);
            if (revision.envelope >= 0 &&
                size_t(revision.envelope) >=
                    program->geometry->weightObjects.size()) {
                return _RrFail(error, bound.mover +
                                          " binds a weight object the "
                                          "file does not carry");
            }
            // The file's operations: Add..Curve in RigExecPropertyOp's
            // order, then Invalid, then Quantize and Path (format 23).
            bound.opValid = true;
            switch (revision.op) {
            case RigExecWirePropertyOp::Add:
            case RigExecWirePropertyOp::Multiply:
            case RigExecWirePropertyOp::Clamp:
            case RigExecWirePropertyOp::Remap:
            case RigExecWirePropertyOp::Blend:
            case RigExecWirePropertyOp::Curve:
                bound.op = RigExecPropertyOp(int(revision.op));
                break;
            case RigExecWirePropertyOp::Quantize:
                bound.op = RigExecPropertyOp::Quantize;
                break;
            case RigExecWirePropertyOp::Path:
                bound.op = RigExecPropertyOp::Path;
                break;
            default:
                bound.opValid = false;
                break;
            }
            if (bound.opValid && bound.op == RigExecPropertyOp::Path) {
                bound.pathPoints = revision.pathPoints;
                bound.path.valid = revision.pathValid;
                bound.path.basis =
                    RigExecPathBasis(uint8_t(revision.pathBasis));
                bound.path.pinned = revision.pathPinned;
                bound.path.planar = revision.pathPlanar;
                bound.path.component =
                    RigExecPathComponent(uint8_t(revision.pathComponent));
                if (revision.pathOffset) {
                    bound.path.offsetX = (*revision.pathOffset)[0];
                    bound.path.offsetZ = (*revision.pathOffset)[1];
                }
                if (revision.pathHeadingOffset) {
                    bound.path.headingOffset = *revision.pathHeadingOffset;
                }
            }
            bound.keys.reserve(revision.keys.size());
            for (const RigExecWireVec2f &key : revision.keys) {
                bound.keys.emplace_back(key[0], key[1]);
            }
            bound.tangents.reserve(revision.tangents.size());
            for (const RigExecWireVec2f &tangent : revision.tangents) {
                bound.tangents.emplace_back(tangent[0], tangent[1]);
            }
            bound.keysValid = RigExecValidateLinearKeysKernel(
                bound.keys.data(), bound.keys.size());
        }
    }
    // Grouped by chain, so each chain's consumers are one range.
    scratch->phasedPublish.resize(file->phasedConsumers.size());
    for (size_t k = 0; k < file->phasedConsumers.size(); ++k) {
        const RigExecWirePhasedConsumer &consumer = file->phasedConsumers[k];
        if (consumer.chain >= scratch->chains.size() ||
            consumer.consumer >= slots) {
            return _RrFail(error, "a phased consumer names no chain or "
                                  "no input slot");
        }
        RrPropertyScratch::Chain &plan = scratch->chains[consumer.chain];
        if (plan.phasedBegin == plan.phasedEnd) {
            plan.phasedBegin = k;
        } else if (plan.phasedEnd != k) {
            return _RrFail(error, "the phased consumers of property "
                                  "chain " +
                                      plan.target + " are not contiguous");
        }
        plan.phasedEnd = k + 1;
        scratch->phasedPublish[k] = entryOf(consumer.consumer);
    }
    const size_t entries = scratch->publishNames.size();
    store.propertyResults.clear();
    store.propertyValues.assign(entries, _RrPropertyZero());
    store.propertyPublished.assign(entries, 0);
    size_t versions = 0;
    for (const auto &chain : file->propertyChains)
        versions = std::max(versions, size_t(chain.versionBase) +
                                          chain.revisions.size() + 1);
    for (const auto &record : file->phasedConsumers)
        versions = std::max(versions, size_t(record.version) + 1);
    store.propertyVersions.assign(versions, _RrPropertyZero());
    store.propertyVersionValid.assign(versions, 0);
    store.propertyVersionChanged.assign(versions, 0);
    store.propertyChainValid.assign(file->propertyChains.size(), 0);
    store.propertyRecordStoodAside.assign(file->phasedConsumers.size(), 0);
    store.propertyPublishedVersions.assign(slots, -1);
    store.propertyPathSlots.clear();
    for (size_t slot = 0; slot < slots; ++slot)
        store.propertyPathSlots.emplace(file->inputs[slot].name(),
                                        uint32_t(slot));
    program->properties = scratch;
    return true;
}

void
RrPropertyBegin(RrProgram *program)
{
    auto &store = program->store;
    store.propertyResults.clear();
    std::fill(store.propertyPublishedVersions.begin(),
              store.propertyPublishedVersions.end(), -1);
    std::fill(store.propertyPublished.begin(), store.propertyPublished.end(),
              char(0));
    std::fill(store.propertyVersionChanged.begin(),
              store.propertyVersionChanged.end(), char(0));
    const auto &records = program->inputState.file->phasedConsumers;
    for (size_t k = 0; k < records.size(); ++k) {
        // The runtime authored API changes the raw upstream layer. It
        // supplies no interactive overlay, so it never stands a record aside.
        const bool standing = false;
        store.propertyRecordStoodAside[k] = standing ? 1 : 0;
        // The producer owns the value; publication validity follows this
        // generation's override kind even while that producer stays clean.
        const char valid =
            store.propertyChainValid[records[k].chain] && !standing;
        if (store.propertyVersionValid[records[k].version] != valid)
            store.propertyVersionChanged[records[k].version] = 1;
        store.propertyVersionValid[records[k].version] = valid;
    }
}

void
RrPropertyPublishFinished(RrProgram *program, const std::vector<char> &finished)
{
    auto &store = program->store;
    const auto &file = *program->inputState.file;
    const auto *scratch =
        static_cast<RrPropertyScratch *>(program->properties.get());
    if (!scratch)
        return;
    for (size_t c = 0; c < file.propertyChains.size(); ++c) {
        if (!finished[c] || !store.propertyChainValid[c])
            continue;
        const auto &chain = file.propertyChains[c];
        const size_t version =
            size_t(chain.versionBase) + chain.revisions.size();
        store.propertyPublishedVersions[chain.target] = int32_t(version);
        _RrPublish(program, *scratch, scratch->chains[c].publish,
                   store.propertyVersions[version]);
        for (size_t k = scratch->chains[c].phasedBegin;
             k < scratch->chains[c].phasedEnd; ++k) {
            const auto &record = file.phasedConsumers[k];
            if (!store.propertyVersionValid[record.version])
                continue;
            store.propertyPublishedVersions[record.consumer] =
                int32_t(record.version);
            _RrPublish(program, *scratch, scratch->phasedPublish[k],
                       store.propertyVersions[record.version]);
        }
    }
}

void
RrPropertyPublish(RrProgram *program)
{
    const std::vector<char> finished(
        program->inputState.file->propertyChains.size(), 1);
    RrPropertyPublishFinished(program, finished);
}

bool
RrRunPropertyPart(RrProgram *program, size_t c, size_t part,
                  std::vector<std::string> *diagnostics)
{
    const auto &file = *program->inputState.file;
    const auto *scratch =
        static_cast<const RrPropertyScratch *>(program->properties.get());
    // The caller's size_t parameters may contain converted wire sentinels.
    if (!scratch || c >= file.propertyChains.size() ||
        c > size_t(std::numeric_limits<int32_t>::max()) ||
        part > size_t(std::numeric_limits<int32_t>::max()))
        return false;
    const eval::StepRef step{uint32_t(eval::StepKind::PropertyRevision),
                             int32_t(c), int32_t(part),
                             size_t(file.propertyChains[c].versionBase) + part};
    RrPropertyHost host{{*program, file, *scratch},
                        {program->store, *diagnostics}};
    return eval::RunValueStep(host, step).outcome != eval::Outcome::Invalid;
}

} // namespace rigExec
