#ifndef RIGEXEC_OPS_FIELD_IO_H
#define RIGEXEC_OPS_FIELD_IO_H

/// Declared payload fields for the op libraries: one field kind per value
/// payloadIo.h saves, with the same bytes, the setters a payloadValues
/// function uses, and the readers a batch function uses. Private to
/// rigExecOps.

#include "rigExec/api/opFields.h"
#include "rigExecStdTypes/stdTypes.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExecOps {
namespace fieldIo {

using rigExec::RigExecFieldDesc;
using rigExec::RigExecFieldValues;
using rigExec::RigExecOpBatch;

/// A value saved as its bytes, as Put, I32, U32, F32 and F64 save it.
inline RigExecFieldDesc
Value(const char *name, const rigExec::RigExecTypeHandle &type)
{
    RigExecFieldDesc f;
    f.name = name;
    f.type = &type;
    return f;
}

/// A flag, as Bool saves it.
inline RigExecFieldDesc
Flag(const char *name)
{
    RigExecFieldDesc f = Value(name, rigExecStd::BoolType());
    f.limit = 1;
    return f;
}

/// An enum, as Enum saves it, read back no greater than \p last.
template <class E>
RigExecFieldDesc
Enum(const char *name, E last)
{
    RigExecFieldDesc f = Value(name, rigExecStd::UIntType());
    f.limit = uint64_t(last);
    return f;
}

/// \p count values saved one after another.
inline RigExecFieldDesc
Fixed(RigExecFieldDesc f, uint32_t count)
{
    f.extent = rigExec::RigExecFieldExtent::Fixed;
    f.count = count;
    return f;
}

/// A list's length, as List saves it ahead of the items.
inline RigExecFieldDesc
Count(const char *name)
{
    return Value(name, rigExecStd::UInt64Type());
}

/// A list's items, whose length is the Count field \p countField.
inline RigExecFieldDesc
Items(const char *name, const rigExec::RigExecTypeHandle &type,
      uint32_t countField)
{
    RigExecFieldDesc f = Value(name, type);
    f.extent = rigExec::RigExecFieldExtent::FromField;
    f.source = countField;
    return f;
}

inline void
SetFlag(RigExecFieldValues &values, uint32_t field, bool flag)
{
    values.Set(field, uint8_t(flag ? 1 : 0));
}

template <class E>
void
SetEnum(RigExecFieldValues &values, uint32_t field, E value)
{
    values.Set(field, uint32_t(value));
}

/// Sets Count field \p countField and the Items field after it.
template <class T>
void
SetList(RigExecFieldValues &values, uint32_t countField,
        const std::vector<T> &items)
{
    values.Set(countField, uint64_t(items.size()));
    values.Set(countField + 1, items);
}

template <class T>
const T &
ValueAt(const RigExecOpBatch &batch, uint32_t field, size_t k)
{
    return rigExec::RigExecPayloadColumnOf<T>(batch, field)[k];
}

inline bool
FlagAt(const RigExecOpBatch &batch, uint32_t field, size_t k)
{
    return ValueAt<uint8_t>(batch, field, k) != 0;
}

template <class E>
E
EnumAt(const RigExecOpBatch &batch, uint32_t field, size_t k)
{
    return E(ValueAt<uint32_t>(batch, field, k));
}

template <class T>
rigExec::RigExecFieldView<const T>
ListAt(const RigExecOpBatch &batch, uint32_t field, size_t k)
{
    return rigExec::RigExecPayloadField<T>(batch, field, k);
}

/// Runs \p one(k) on each active instance in order; at the first false it
/// sets \p failed to that instance and returns false.
template <class One>
bool
EachInstance(const RigExecOpBatch &batch, size_t *failed, One one)
{
    for (size_t k = 0; k < batch.count; ++k) {
        if (rigExec::RigExecBatchActive(batch, k) && !one(k)) {
            *failed = k;
            return false;
        }
    }
    return true;
}

} // namespace fieldIo
} // namespace rigExecOps

#endif
