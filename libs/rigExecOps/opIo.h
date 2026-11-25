#ifndef RIGEXEC_OPS_OP_IO_H
#define RIGEXEC_OPS_OP_IO_H

/// Port access and the point ops' shared envelope handling. Private to
/// rigExecOps.

#include "rigExec/api/opPorts.h"
#include "rigExec/api/registry.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/smoothKernel.h"
#include "rigExecStdTypes/stdTypes.h"

#include <algorithm>
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

/// What validate knows of input \p port (RigExecOpCheck): its elements,
/// and its value when the host writes it.
inline size_t
CheckElements(const rigExec::RigExecOpCheck &check,
              const rigExec::RigExecPortHandle &port)
{
    return check.elements[port.Index()];
}

template <class T>
const T *
CheckValue(const rigExec::RigExecOpCheck &check,
           const rigExec::RigExecPortHandle &port)
{
    return static_cast<const T *>(
        check.batch.inputs[check.k * check.batch.portCount + port.Index()]);
}

inline bool
Fed(const rigExec::RigExecOpCheck &check,
    const rigExec::RigExecPortHandle &port)
{
    return CheckElements(check, port) != rigExec::kRigExecPortUnfed;
}

/// Whether \p a and \p b are equal where both are known.
inline bool
SameCount(size_t a, size_t b)
{
    return a >= rigExec::kRigExecPortUnfed || b >= rigExec::kRigExecPortUnfed ||
           a == b;
}

/// The node's working state as validate sees it; null at compile.
template <class State>
State *
CheckState(const rigExec::RigExecOpCheck &check)
{
    return static_cast<State *>(check.state);
}

/// Whether \p envelope, when fed, can hold one weight per point.
inline bool
EnvelopeFits(const rigExec::RigExecOpCheck &check,
             const rigExec::RigExecPortHandle &points,
             const rigExec::RigExecPortHandle &envelope, const char *op,
             std::string *error)
{
    if (!SameCount(CheckElements(check, envelope),
                   CheckElements(check, points)))
        return Fail(error, op, "envelope length differs from the points");
    return true;
}

/// Whether \p envelope, when given, holds one weight per point.
inline bool
EnvelopeFits(const Points &in, const Envelope *envelope, const char *op,
             std::string *error)
{
    if (envelope && envelope->Size() != in.Size())
        return Fail(error, op, "envelope length differs from the points");
    return true;
}

/// Blends points [begin, end) of \p out back toward \p in by \p envelope;
/// null is 1 everywhere.
inline void
BlendEnvelopeRange(const Points &in, const Envelope *envelope, Points &out,
                   size_t begin, size_t end)
{
    if (!envelope)
        return;
    for (size_t i = begin; i < end; ++i)
        out[i] = rigExec::RigExecBlendEnvelope(in[i], out[i], (*envelope)[i]);
}

/// Blends \p out back toward \p in by \p envelope; null is 1 everywhere.
inline bool
BlendEnvelope(const Points &in, const Envelope *envelope, Points *out,
              const char *op, std::string *error)
{
    if (!EnvelopeFits(in, envelope, op, error))
        return false;
    BlendEnvelopeRange(in, envelope, *out, 0, in.Size());
    return true;
}

/// Copies points [begin, end) of \p in to \p out.
inline void
CopyRange(const Points &in, Points &out, size_t begin, size_t end)
{
    std::copy(in.Data() + begin, in.Data() + end, out.Data() + begin);
}

/// Element range.begin of input \p port of a ranged node, then the rest of
/// the range's: through RigExecOpRange::at for a pointwise port, else in
/// the whole value; null for an unfed or empty input.
template <class T>
const T *
RangeIn(const rigExec::RigExecOpIo &io, const rigExec::RigExecOpRange &range,
        const rigExec::RigExecPortHandle &port)
{
    if (range.at && range.at[port.Index()])
        return static_cast<const T *>(range.at[port.Index()]);
    const auto *whole = In<rigExecStd::RigExecArray<T>>(io, port);
    return whole && whole->Size() ? whole->Data() + range.begin : nullptr;
}

/// Element range.begin of pointwise output \p port, then the rest of the
/// range's.
template <class T>
T *
RangeOut(const rigExec::RigExecOpIo &io, const rigExec::RigExecOpRange &range,
         const rigExec::RigExecPortHandle &port)
{
    if (range.at && range.at[port.Index()])
        return static_cast<T *>(range.at[port.Index()]);
    return Out<rigExecStd::RigExecArray<T>>(io, port).Data() + range.begin;
}

/// The groups of neighbourhood input \p port a range reads.
template <class V>
rigExec::RigExecGroupPoints<V>
RangeGroups(const rigExec::RigExecOpRange &range,
            const rigExec::RigExecPortHandle &port)
{
    rigExec::RigExecGroupPoints<V> out;
    const auto *groups = range.at
                             ? static_cast<const rigExec::RigExecOpGroups *>(
                                   range.at[port.Index()])
                             : nullptr;
    if (!groups)
        return out;
    out.bounds = groups->bounds;
    out.groups = groups->groups;
    out.points = reinterpret_cast<const V *const *>(groups->points);
    return out;
}

/// Blends the \p count points at \p out back toward \p in by \p envelope;
/// null is 1 everywhere.
inline void
BlendEnvelopeAt(const rigExecStd::Vec3f *in, const float *envelope,
                rigExecStd::Vec3f *out, size_t count)
{
    if (!envelope)
        return;
    for (size_t i = 0; i < count; ++i)
        out[i] = rigExec::RigExecBlendEnvelope(in[i], out[i], envelope[i]);
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
