#ifndef RIGEXEC_OPS_OP_IO_H
#define RIGEXEC_OPS_OP_IO_H

/// Port access and the point ops' shared envelope handling. Private to
/// rigExecOps.

#include "rigExec/api/opPorts.h"
#include "rigExec/api/registry.h"
#include "rigExecMath/envelope.h"
#include "rigExecStdTypes/stdTypes.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace rigExecOps {
namespace opIo {

using Points = rigExecStd::RigExecArray<rigExecStd::Vec3f>;
using Envelope = rigExecStd::RigExecArray<float>;

using rigExec::opPorts::In;
using rigExec::opPorts::InList;
using rigExec::opPorts::InOr;
using rigExec::opPorts::Out;

/// State handling for an op whose per-node working state is a \p State,
/// constructed when an evaluator binds the program and kept until it
/// unbinds.
template <class State>
rigExec::RigExecOpStateHandling
StateOf()
{
    rigExec::RigExecOpStateHandling h;
    h.size = sizeof(State);
    h.align = alignof(State);
    h.create = [](void *dst) {
        std::uninitialized_value_construct_n(static_cast<State *>(dst), 1);
    };
    h.copy = [](void *dst, const void *src) {
        std::uninitialized_copy_n(static_cast<const State *>(src), 1,
                                  static_cast<State *>(dst));
    };
    h.reset = [](void *dst) { *static_cast<State *>(dst) = State(); };
    h.destroy = [](void *dst) { std::destroy_at(static_cast<State *>(dst)); };
    return h;
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

/// Whether every point is finite.
inline bool
AllFinite(const std::vector<rigExecStd::Vec3f> &points)
{
    for (const rigExecStd::Vec3f &p : points)
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
            !std::isfinite(p[2]))
            return false;
    return true;
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
    out.Assign(points.data(), points.size());
    return BlendEnvelope(in, envelope, &out, op, error);
}

} // namespace opIo
} // namespace rigExecOps

#endif
