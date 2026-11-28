#ifndef RIGEXEC_API_OP_PORTS_H
#define RIGEXEC_API_OP_PORTS_H

/// Typed access to port values inside an op's run function: read an input
/// or write an output by its port handle and value type.

#include "rigExec/api/registry.h"

#include <cstddef>

namespace rigExec {
namespace opPorts {

/// An input's value; null for an unfed optional input.
template <class T>
const T *
In(const RigExecOpIo &io, const RigExecPortHandle &port)
{
    return static_cast<const T *>(io.inputs[port.Index()]);
}

/// The values of an input that takes many, in the order they were fed.
template <class T> class InList {
public:
    InList(const RigExecOpIo &io, const RigExecPortHandle &port)
        : _list(static_cast<const RigExecPortList *>(io.inputs[port.Index()]))
    {
    }
    size_t Size() const
    {
        return _list->count;
    }
    const T &operator[](size_t i) const
    {
        return *static_cast<const T *>(_list->items[i]);
    }

private:
    const RigExecPortList *_list;
};

/// An input's value, or \p fallback when the optional input is unfed.
template <class T>
T
InOr(const RigExecOpIo &io, const RigExecPortHandle &port, const T &fallback)
{
    const T *value = In<T>(io, port);
    return value ? *value : fallback;
}

/// The output value to write.
template <class T>
T &
Out(const RigExecOpIo &io, const RigExecPortHandle &port)
{
    return *static_cast<T *>(io.outputs[port.Index()]);
}

} // namespace opPorts
} // namespace rigExec

#endif
