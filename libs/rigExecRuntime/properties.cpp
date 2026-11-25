// Bind playback property parts to decoded reads and typed result slots.
#include "rigExecRuntime/store.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecEvalCore/propertyLookups.h"
#include "rigExecMath/propertyMathKernel.h"

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

template <class T>
bool
_RrPutPort(const T &value, const RigExecTypeDesc &type, void *out)
{
    if (type.size != sizeof(T) || !type.triviallyCopyable)
        return false;
    std::memcpy(out, &value, sizeof(T));
    return true;
}

template <class T>
bool
_RrBorrowPort(const T *elements, size_t count, const RigExecTypeDesc &type,
              void *out)
{
    if (!type.borrow)
        return false;
    type.borrow(out, elements, count);
    return true;
}

bool
_RrBaseFinite(float value)
{
    return eval::PropertyFinite(value);
}
bool
_RrBaseFinite(double value)
{
    return eval::PropertyFinite(value);
}
bool
_RrBaseFinite(const RrVec3f &value)
{
    const float elements[3] = {value[0], value[1], value[2]};
    return eval::PropertyFinite(elements, 3);
}
bool
_RrBaseFinite(const RrMat4d &value)
{
    for (size_t r = 0; r < 4; ++r) {
        const double row[4] = {value[r][0], value[r][1], value[r][2],
                               value[r][3]};
        if (!eval::PropertyFinite(row, 4))
            return false;
    }
    return true;
}

template <class T>
bool
_RrLookUpBase(const RrProgram &program, const RrPropertyScratch &scratch,
              size_t c, const RigExecTypeDesc &type, void *out,
              std::string *error)
{
    const std::string label = "property chain " + scratch.chains[c].target;
    RrWireValue base;
    if (!RrChainBase(&program, c, scratch.chains[c].baseTag, &base))
        return _RrFail(error, label + ": target has no authored value");
    T value;
    if constexpr (std::is_same_v<T, float>)
        value = RrWireValueFloat(base);
    else if constexpr (std::is_same_v<T, double>)
        std::memcpy(&value, &base.bits, sizeof(value));
    else if constexpr (std::is_same_v<T, RrMat4d>)
        value = _RrMatrix(base.matrix);
    else
        value = _RrVec3f(base);
    if (!_RrBaseFinite(value))
        return _RrFail(error, label + ": authored base is not finite");
    return _RrPutPort(value, type, out) ||
           _RrFail(error, label + ": the base does not fit its op input");
}

bool
_RrEnabled(const RrProgram &program, const RigExecWirePropertyRevision &wire)
{
    return RrReadInput(&program, *wire.enabled).bits != 0;
}

template <class T>
void
_RrStoreVersion(RrStore &store, uint32_t id, const void *value)
{
    T next;
    std::memcpy(&next, value, sizeof(T));
    store.propertyVersions[id] = _RrHold(next);
    store.propertyVersionValid[id] = 1;
}

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
RrLookUpProperty(const RrProgram *program, const eval::OpLookup &lookup,
                 const RigExecTypeDesc &type, void *out, std::string *error)
{
    const auto &file = *program->inputState.file;
    const auto *scratch =
        static_cast<const RrPropertyScratch *>(program->properties.get());
    if (!scratch || lookup.object >= file.propertyChains.size())
        return _RrFail(error, "op nodes: no property chain " +
                                  std::to_string(lookup.object));
    const size_t c = lookup.object;
    const RigExecWirePropertyChain &chain = file.propertyChains[c];
    using eval::PropertyRead;
    const PropertyRead read = PropertyRead(lookup.read);
    if (read == PropertyRead::Base) {
        switch (chain.valueType) {
        case RigExecWirePropertyValueType::Float:
            return _RrLookUpBase<float>(*program, *scratch, c, type, out,
                                        error);
        case RigExecWirePropertyValueType::Double:
            return _RrLookUpBase<double>(*program, *scratch, c, type, out,
                                         error);
        case RigExecWirePropertyValueType::Matrix4d:
            return _RrLookUpBase<RrMat4d>(*program, *scratch, c, type, out,
                                          error);
        case RigExecWirePropertyValueType::Vec3f:
            return _RrLookUpBase<RrVec3f>(*program, *scratch, c, type, out,
                                          error);
        default:
            return _RrFail(error, "property chain " +
                                      scratch->chains[c].target +
                                      " has an unknown value type");
        }
    }
    if (lookup.part < 0 || size_t(lookup.part) >= chain.revisions.size())
        return _RrFail(error, "property chain " + scratch->chains[c].target +
                                  " has no revision " +
                                  std::to_string(lookup.part));
    const RigExecWirePropertyRevision &wire =
        chain.revisions[size_t(lookup.part)];
    const RrPropertyScratch::Revision &bound =
        scratch->chains[c].revisions[size_t(lookup.part)];
    const auto operand = [&](const RigExecWireInput &input) {
        const RrWireValue value = RrReadInput(program, input);
        switch (chain.valueType) {
        case RigExecWirePropertyValueType::Matrix4d:
            return _RrPutPort(_RrMatrix(value.matrix), type, out);
        case RigExecWirePropertyValueType::Vec3f:
            return _RrPutPort(_RrVec3f(value), type, out);
        default:
            return _RrPutPort(RrWireValueFloat(value), type, out);
        }
    };
    static_assert(sizeof(RrVec2f) == 2 * sizeof(float) &&
                      sizeof(RrVec3f) == 3 * sizeof(float),
                  "keys and points are packed floats");
    bool put = false;
    switch (read) {
    case PropertyRead::Base:
        break;
    case PropertyRead::Enabled:
        put =
            _RrPutPort(uint8_t(_RrEnabled(*program, wire) ? 1 : 0), type, out);
        break;
    case PropertyRead::Weight: {
        float weight = 1.0f;
        if (!_RrEnabled(*program, wire)) {
        } else if (wire.envelope >= 0) {
            const auto &field =
                program->store.weightFieldResults[size_t(wire.weightField)];
            if (!field.ok)
                return _RrFail(error, field.error);
            if (field.values.size() != 1)
                return _RrFail(error, "the weight object holds " +
                                          std::to_string(field.values.size()) +
                                          " weights, not one");
            weight = field.values[0];
        } else {
            weight =
                RrWireValueFloat(RrReadInput(program, *wire.defaultWeight));
        }
        put = _RrPutPort(weight, type, out);
        break;
    }
    case PropertyRead::Value:
        put = operand(*wire.value);
        break;
    case PropertyRead::Minimum:
        put = operand(*wire.min);
        break;
    case PropertyRead::Maximum:
        put = operand(*wire.max);
        break;
    case PropertyRead::Keys:
        put = _RrBorrowPort(bound.keys.data(), bound.keys.size(), type, out);
        break;
    case PropertyRead::Tangents:
        put = _RrBorrowPort(bound.tangents.data(), bound.tangents.size(), type,
                            out);
        break;
    case PropertyRead::PathPoints: {
        const auto *points =
            bound.pathPoints >= 0
                ? RrInputArray<RrVec3f>(program, uint32_t(bound.pathPoints))
                : nullptr;
        put = points ? _RrBorrowPort(points->data(), points->size(), type, out)
                     : _RrBorrowPort<RrVec3f>(nullptr, 0, type, out);
        break;
    }
    }
    return put || _RrFail(error, "a read does not fit its op input");
}

std::string
RrPropertyRevisionLabel(const RrProgram *program, size_t chain, size_t part)
{
    const auto *scratch =
        static_cast<const RrPropertyScratch *>(program->properties.get());
    return scratch ? "diag " + scratch->chains[chain].revisions[part - 1].mover
                   : std::string();
}

bool
RrPropertyPartOfVersion(const RrProgram *program, uint32_t version,
                        size_t *chain, size_t *part)
{
    const auto &chains = program->file->propertyChains;
    for (size_t c = 0; c < chains.size(); ++c) {
        const uint32_t base = chains[c].versionBase;
        if (version >= base && version <= base + chains[c].revisions.size()) {
            *chain = c;
            *part = version - base;
            return true;
        }
    }
    return false;
}

bool
RrStorePropertyVersion(RrProgram *program, uint32_t version, const void *value,
                       std::string *error)
{
    const auto &file = *program->inputState.file;
    const auto *scratch =
        static_cast<const RrPropertyScratch *>(program->properties.get());
    size_t c = 0, part = 0;
    if (!scratch || version >= program->store.propertyVersions.size() ||
        !RrPropertyPartOfVersion(program, version, &c, &part))
        return _RrFail(error, "op nodes: no property version " +
                                  std::to_string(version));
    RrStore &store = program->store;
    switch (file.propertyChains[c].valueType) {
    case RigExecWirePropertyValueType::Float:
        _RrStoreVersion<float>(store, version, value);
        break;
    case RigExecWirePropertyValueType::Double:
        _RrStoreVersion<double>(store, version, value);
        break;
    case RigExecWirePropertyValueType::Matrix4d:
        _RrStoreVersion<RrMat4d>(store, version, value);
        break;
    case RigExecWirePropertyValueType::Vec3f:
        _RrStoreVersion<RrVec3f>(store, version, value);
        break;
    default:
        return _RrFail(error, "property chain " + scratch->chains[c].target +
                                  " has an unknown value type");
    }
    if (part == 0)
        store.propertyChainValid[c] = 1;
    const RrPropertyValue &stored = store.propertyVersions[version];
    for (size_t k = scratch->chains[c].phasedBegin;
         k < scratch->chains[c].phasedEnd; ++k) {
        const RigExecWirePhasedConsumer &record = file.phasedConsumers[k];
        if (record.applied != part)
            continue;
        RrPropertyValue converted = stored;
        if (stored.tag == _RrTag::Float &&
            record.consumerType == RigExecWirePropertyValueType::Double)
            converted = _RrHold(double(stored.f32));
        else if (stored.tag == _RrTag::Double &&
                 record.consumerType == RigExecWirePropertyValueType::Float)
            converted = _RrHold(float(stored.f64));
        store.propertyVersions[record.version] = converted;
        store.propertyVersionValid[record.version] =
            !store.propertyRecordStoodAside[k];
    }
    return true;
}

} // namespace rigExec
