#ifndef RIGEXEC_OPS_PAYLOAD_IO_H
#define RIGEXEC_OPS_PAYLOAD_IO_H

/// Saving and reading the values op payloads are made of. Private to
/// rigExecOps.

#include "rigExec/api/payloadCodec.h"
#include "rigExecStdTypes/stdTypes.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExecOps {
namespace payloadIo {

using Writer = rigExec::RigExecPayloadWriter;
using Reader = rigExec::RigExecPayloadReader;

inline void
Put(Writer &w, uint8_t v)
{
    w.U8(v);
}
inline void
Get(Reader &r, uint8_t &v)
{
    v = r.U8();
}
inline void
Put(Writer &w, int32_t v)
{
    w.I32(v);
}
inline void
Get(Reader &r, int32_t &v)
{
    v = r.I32();
}
inline void
Put(Writer &w, uint32_t v)
{
    w.U32(v);
}
inline void
Get(Reader &r, uint32_t &v)
{
    v = r.U32();
}
inline void
Put(Writer &w, float v)
{
    w.F32(v);
}
inline void
Get(Reader &r, float &v)
{
    v = r.F32();
}
inline void
Put(Writer &w, double v)
{
    w.F64(v);
}
inline void
Get(Reader &r, double &v)
{
    v = r.F64();
}
inline void
Put(Writer &w, const rigExecStd::Vec2f &v)
{
    w.F32(v[0]);
    w.F32(v[1]);
}
inline void
Get(Reader &r, rigExecStd::Vec2f &v)
{
    v[0] = r.F32();
    v[1] = r.F32();
}
inline void
Put(Writer &w, const rigExecStd::Vec3f &v)
{
    for (int i = 0; i < 3; ++i)
        w.F32(v[i]);
}
inline void
Get(Reader &r, rigExecStd::Vec3f &v)
{
    for (int i = 0; i < 3; ++i)
        v[i] = r.F32();
}
inline void
Put(Writer &w, const rigExecStd::Vec3d &v)
{
    for (int i = 0; i < 3; ++i)
        w.F64(v[i]);
}
inline void
Get(Reader &r, rigExecStd::Vec3d &v)
{
    for (int i = 0; i < 3; ++i)
        v[i] = r.F64();
}
inline void
Put(Writer &w, const rigExecStd::Mat4d &m)
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            w.F64(m[i][j]);
}
inline void
Get(Reader &r, rigExecStd::Mat4d &m)
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m[i][j] = r.F64();
}
inline void
PutFlags(Writer &w, const std::array<bool, 3> &flags)
{
    for (bool f : flags)
        w.Bool(f);
}
inline void
GetFlags(Reader &r, std::array<bool, 3> &flags)
{
    for (bool &f : flags)
        f = r.Bool();
}

/// The bytes one item of \p T takes.
template <class T> constexpr size_t kBytes = sizeof(T);

template <class T>
void
PutList(Writer &w, const std::vector<T> &items)
{
    w.List(items, [](Writer &out, const T &item) { Put(out, item); });
}
template <class T>
void
GetList(Reader &r, std::vector<T> &items)
{
    r.List(items, kBytes<T>, [](Reader &in, T &item) { Get(in, item); });
}

} // namespace payloadIo
} // namespace rigExecOps

#endif
