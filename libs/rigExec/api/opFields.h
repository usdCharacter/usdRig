#ifndef RIGEXEC_API_OP_FIELDS_H
#define RIGEXEC_API_OP_FIELDS_H

/// Declared op fields (RigExecFieldDesc): the values a host gives a node,
/// how a batch function reads them, and how they are checked, saved and
/// read back. One implementation serves every op that declares fields.

#include "rigExec/api/registry.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

namespace rigExec {

/// One node's values of an op's declared payload fields: per field, its
/// values' bytes and how many values there are.
class RigExecFieldValues {
public:
    RigExecFieldValues() = default;
    explicit RigExecFieldValues(size_t fieldCount)
        : _bytes(fieldCount), _counts(fieldCount, 0)
    {
    }

    size_t FieldCount() const
    {
        return _counts.size();
    }
    /// Replaces field \p field with \p count values of \p valueSize bytes
    /// each, copied from \p data.
    void SetBytes(size_t field, const void *data, size_t count,
                  size_t valueSize);
    template <class T> void Set(size_t field, const T *data, size_t count)
    {
        static_assert(std::is_trivially_copyable_v<T>,
                      "field values are plain values");
        SetBytes(field, data, count, sizeof(T));
    }
    template <class T> void Set(size_t field, const T &value)
    {
        Set(field, &value, 1);
    }
    template <class T> void Set(size_t field, const std::vector<T> &values)
    {
        Set(field, values.data(), values.size());
    }
    const std::byte *Data(size_t field) const
    {
        return _bytes[field].data();
    }
    size_t Count(size_t field) const
    {
        return _counts[field];
    }
    size_t ByteSize(size_t field) const
    {
        return _bytes[field].size();
    }
    /// Equal when every field holds the same count and the same bytes.
    bool operator==(const RigExecFieldValues &other) const
    {
        return _counts == other._counts && _bytes == other._bytes;
    }
    bool operator!=(const RigExecFieldValues &other) const
    {
        return !(*this == other);
    }
    uint64_t Hash() const;

private:
    std::vector<std::vector<std::byte>> _bytes;
    std::vector<size_t> _counts;
};

/// Checks \p values against \p op's payload fields: one entry per field,
/// each with the count its extent asks for, of its type's size, and no
/// value above its limit.
bool RigExecCheckPayloadFields(const RigExecOpDesc &op,
                               const RigExecFieldValues &values,
                               std::string *error = nullptr);

/// Saves checked \p values of \p op: payloadVersion when it is not 0, then
/// each field's values in order, a FromInput field's preceded by its count.
void RigExecEncodePayloadFields(const RigExecOpDesc &op,
                                const RigExecFieldValues &values,
                                std::vector<uint8_t> &bytes);

/// Reads \p size bytes RigExecEncodePayloadFields wrote into \p values;
/// false with the reason in \p error for bytes it cannot have written.
bool RigExecDecodePayloadFields(const RigExecOpDesc &op, const uint8_t *bytes,
                                size_t size, RigExecFieldValues *values,
                                std::string *error = nullptr);

/// Saves \p payload, the settings struct a host passes to AddOp, whether
/// \p op declares its fields or handles its payload itself.
bool RigExecEncodeOpPayload(const RigExecOpDesc &op, const void *payload,
                            std::vector<uint8_t> &bytes,
                            std::string *error = nullptr);

/// A field's values for one instance.
template <class T> struct RigExecFieldView {
    T *data = nullptr;
    size_t count = 0;

    size_t Size() const
    {
        return count;
    }
    bool Empty() const
    {
        return count == 0;
    }
    T &operator[](size_t i) const
    {
        return data[i];
    }
    T *begin() const
    {
        return data;
    }
    T *end() const
    {
        return data + count;
    }
};

template <class T, class Byte>
RigExecFieldView<T>
RigExecFieldAt(const RigExecFieldColumnOf<Byte> &column, size_t k)
{
    if (column.runs)
        return {reinterpret_cast<T *>(column.runs[k].data),
                column.runs[k].count};
    return {reinterpret_cast<T *>(column.base + k * column.stride),
            column.count};
}

/// Instance \p k's values of payload field \p field.
template <class T>
RigExecFieldView<const T>
RigExecPayloadField(const RigExecOpBatch &batch, uint32_t field, size_t k)
{
    return RigExecFieldAt<const T>(batch.payloadFields[field], k);
}

/// Every instance's values of Scalar or Fixed payload field \p field as
/// one array: instance k's start at k times the field's count.
template <class T>
const T *
RigExecPayloadColumnOf(const RigExecOpBatch &batch, uint32_t field)
{
    return reinterpret_cast<const T *>(batch.payloadFields[field].base);
}

/// Instance \p k's values of state field \p field.
template <class T>
RigExecFieldView<T>
RigExecStateField(const RigExecOpBatch &batch, uint32_t field, size_t k)
{
    return RigExecFieldAt<T>(batch.stateFields[field], k);
}

/// Every instance's values of Scalar or Fixed state field \p field as one
/// array.
template <class T>
T *
RigExecStateColumnOf(const RigExecOpBatch &batch, uint32_t field)
{
    return reinterpret_cast<T *>(batch.stateFields[field].base);
}

} // namespace rigExec

#endif
