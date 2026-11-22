#ifndef RIGEXEC_OPS_OP_IO_H
#define RIGEXEC_OPS_OP_IO_H

/// Port access and the point ops' shared envelope handling. Private to
/// rigExecOps.

#include "rigExec/api/registry.h"
#include "rigExecMath/envelope.h"
#include "rigExecStdTypes/stdTypes.h"

#include <string>
#include <vector>

namespace rigExecOps {
namespace opIo {

using Points = rigExecStd::RigExecArray<rigExecStd::Vec3f>;
using Envelope = rigExecStd::RigExecArray<float>;

/// An input's value; null for an unfed optional input.
template <class T>
const T *
In(const rigExec::RigExecOpIo &io, const rigExec::RigExecPortHandle &port)
{
    return static_cast<const T *>(io.inputs[port.Index()]);
}

/// An array input's elements, in feed order.
template <class T> class InList {
public:
    InList(const rigExec::RigExecOpIo &io,
           const rigExec::RigExecPortHandle &port)
        : _list(static_cast<const rigExec::RigExecPortList *>(
              io.inputs[port.Index()]))
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
    const rigExec::RigExecPortList *_list;
};

template <class T>
T
InOr(const rigExec::RigExecOpIo &io, const rigExec::RigExecPortHandle &port,
     const T &fallback)
{
    const T *value = In<T>(io, port);
    return value ? *value : fallback;
}

template <class T>
T &
Out(const rigExec::RigExecOpIo &io, const rigExec::RigExecPortHandle &port)
{
    return *static_cast<T *>(io.outputs[port.Index()]);
}

template <class T>
std::vector<T>
ToVector(const rigExecStd::RigExecArray<T> &a)
{
    return std::vector<T>(a.begin(), a.end());
}

inline bool
Fail(std::string *error, const char *op, const std::string &why)
{
    if (error)
        *error = std::string(op) + ": " + why;
    return false;
}

/// Blends \p out back toward \p in by \p envelope; null is 1 everywhere.
inline bool
BlendEnvelope(const Points &in, const Envelope *envelope, Points *out,
              const char *op, std::string *error)
{
    if (!envelope)
        return true;
    if (envelope->Size() != in.Size())
        return Fail(error, op, "envelope length differs from the points");
    for (size_t i = 0; i < in.Size(); ++i)
        (*out)[i] =
            rigExec::RigExecBlendEnvelope(in[i], (*out)[i], (*envelope)[i]);
    return true;
}

/// Writes \p points to \p port, blended toward \p in by \p envelope.
inline bool
WritePoints(const rigExec::RigExecOpIo &io,
            const rigExec::RigExecPortHandle &port, const Points &in,
            const std::vector<rigExecStd::Vec3f> &points,
            const Envelope *envelope, const char *op, std::string *error)
{
    Points &out = Out<Points>(io, port);
    out = Points(points.data(), points.size());
    return BlendEnvelope(in, envelope, &out, op, error);
}

} // namespace opIo
} // namespace rigExecOps

#endif
