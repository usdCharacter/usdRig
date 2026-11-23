#include "rigExec/api/opFields.h"
#include "rigExec/api/payloadCodec.h"

#include <cstring>

namespace rigExec {

namespace {

bool
Fail(std::string *error, std::string message)
{
    if (error)
        *error = std::move(message);
    return false;
}

size_t
ValueSize(const RigExecFieldDesc &field)
{
    return RigExecRegistry::Get().Type(*field.type).desc->size;
}

/// The unsigned integer of \p size bytes at \p data, stored little-endian.
uint64_t
UnsignedAt(const std::byte *data, size_t size)
{
    uint64_t value = 0;
    for (size_t i = 0; i < size; ++i)
        value |= uint64_t(data[i]) << (8 * i);
    return value;
}

/// The count field \p f's extent asks for, read from \p values; false for
/// a FromInput field, whose count the values decide.
bool
ExtentCount(const RigExecOpDesc &op, const RigExecFieldValues &values, size_t f,
            uint64_t *count)
{
    const RigExecFieldDesc &field = op.payloadFields[f];
    switch (field.extent) {
    case RigExecFieldExtent::Scalar:
        *count = 1;
        return true;
    case RigExecFieldExtent::Fixed:
        *count = field.count;
        return true;
    case RigExecFieldExtent::FromField: {
        const RigExecFieldDesc &source = op.payloadFields[field.source];
        *count = UnsignedAt(values.Data(field.source), ValueSize(source));
        return true;
    }
    case RigExecFieldExtent::FromInput:
        break;
    }
    return false;
}

/// Whether none of the \p count values at \p data is above the field's
/// limit.
bool
WithinLimit(const RigExecFieldDesc &field, const std::byte *data, size_t count)
{
    if (field.limit == UINT64_MAX)
        return true;
    const size_t size = ValueSize(field);
    for (size_t i = 0; i < count; ++i)
        if (UnsignedAt(data + i * size, size) > field.limit)
            return false;
    return true;
}

std::string
FieldText(const RigExecOpDesc &op, size_t f)
{
    return std::string(op.name ? op.name : "") + " field " +
           (op.payloadFields[f].name ? op.payloadFields[f].name : "");
}

} // namespace

void
RigExecFieldValues::SetBytes(size_t field, const void *data, size_t count,
                             size_t valueSize)
{
    const auto *from = static_cast<const std::byte *>(data);
    _bytes[field].assign(from, from + count * valueSize);
    _counts[field] = count;
}

uint64_t
RigExecFieldValues::Hash() const
{
    uint64_t h = 1469598103934665603ull;
    const auto mix = [&h](const void *data, size_t size) {
        const auto *bytes = static_cast<const unsigned char *>(data);
        for (size_t i = 0; i < size; ++i) {
            h ^= bytes[i];
            h *= 1099511628211ull;
        }
    };
    for (size_t f = 0; f < _counts.size(); ++f) {
        mix(&_counts[f], sizeof(size_t));
        mix(_bytes[f].data(), _bytes[f].size());
    }
    return h;
}

bool
RigExecCheckPayloadFields(const RigExecOpDesc &op,
                          const RigExecFieldValues &values, std::string *error)
{
    if (values.FieldCount() != op.payloadFieldCount)
        return Fail(error, std::string(op.name ? op.name : "") + " takes " +
                               std::to_string(op.payloadFieldCount) +
                               " fields, not " +
                               std::to_string(values.FieldCount()));
    for (size_t f = 0; f < op.payloadFieldCount; ++f) {
        const RigExecFieldDesc &field = op.payloadFields[f];
        const size_t count = values.Count(f);
        uint64_t expected = 0;
        if (ExtentCount(op, values, f, &expected) && expected != count)
            return Fail(error, FieldText(op, f) + " holds " +
                                   std::to_string(count) + " values, not " +
                                   std::to_string(expected));
        if (values.ByteSize(f) != count * ValueSize(field))
            return Fail(error,
                        FieldText(op, f) + " holds values of another size");
        if (!WithinLimit(field, values.Data(f), count))
            return Fail(error, FieldText(op, f) + ": a value is out of range");
    }
    return true;
}

void
RigExecEncodePayloadFields(const RigExecOpDesc &op,
                           const RigExecFieldValues &values,
                           std::vector<uint8_t> &bytes)
{
    bytes.clear();
    RigExecPayloadWriter writer(bytes);
    if (op.payloadVersion)
        writer.Version(op.payloadVersion);
    for (size_t f = 0; f < op.payloadFieldCount; ++f) {
        if (op.payloadFields[f].extent == RigExecFieldExtent::FromInput)
            writer.Count(values.Count(f));
        writer.Bytes(values.Data(f), values.ByteSize(f));
    }
}

bool
RigExecDecodePayloadFields(const RigExecOpDesc &op, const uint8_t *bytes,
                           size_t size, RigExecFieldValues *values,
                           std::string *error)
{
    RigExecPayloadReader reader(bytes, size);
    if (op.payloadVersion)
        reader.Version(op.payloadVersion);
    RigExecFieldValues out(op.payloadFieldCount);
    std::vector<std::byte> scratch;
    for (size_t f = 0; f < op.payloadFieldCount && reader.Ok(); ++f) {
        const RigExecFieldDesc &field = op.payloadFields[f];
        const size_t valueSize = ValueSize(field);
        uint64_t count = 0;
        if (!ExtentCount(op, out, f, &count))
            count = reader.Count(valueSize);
        else if (field.extent == RigExecFieldExtent::FromField &&
                 count > reader.Remaining() / valueSize)
            reader.Fail("a list is longer than the bytes left");
        if (!reader.Ok())
            break;
        scratch.resize(size_t(count) * valueSize);
        reader.Bytes(scratch.data(), scratch.size());
        if (!reader.Ok())
            break;
        if (!WithinLimit(field, scratch.data(), size_t(count)))
            return Fail(error, FieldText(op, f) + ": a value is out of range");
        out.SetBytes(f, scratch.data(), size_t(count), valueSize);
    }
    if (!reader.Finish(error))
        return false;
    *values = std::move(out);
    return true;
}

bool
RigExecEncodeOpPayload(const RigExecOpDesc &op, const void *payload,
                       std::vector<uint8_t> &bytes, std::string *error)
{
    const std::string name = op.name ? op.name : "";
    if (op.payloadFieldCount) {
        if (!payload || !op.payloadValues)
            return Fail(error, "op " + name + " needs its field values");
        RigExecFieldValues values(op.payloadFieldCount);
        op.payloadValues(payload, values);
        if (!RigExecCheckPayloadFields(op, values, error))
            return false;
        RigExecEncodePayloadFields(op, values, bytes);
        return true;
    }
    bytes.clear();
    if (!op.payload.size)
        return true;
    if (!payload || !op.payload.encode)
        return Fail(error, "op " + name +
                               (payload ? " cannot save its settings"
                                        : " needs its settings"));
    op.payload.encode(payload, bytes);
    return true;
}

} // namespace rigExec
