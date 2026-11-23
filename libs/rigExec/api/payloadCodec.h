#ifndef RIGEXEC_API_PAYLOAD_CODEC_H
#define RIGEXEC_API_PAYLOAD_CODEC_H

/// Writes an op's settings ("payload") as bytes and reads them back, so a
/// saved rig can carry them. Numbers are stored little-endian on every
/// machine; a list is stored as its length, then its items. The reader
/// never reads past the end of its bytes: the first problem stops it and
/// leaves an error message.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace rigExec {

/// Appends values to a byte buffer.
class RigExecPayloadWriter {
public:
    explicit RigExecPayloadWriter(std::vector<uint8_t> &bytes) : _bytes(bytes)
    {
    }

    /// Starts a payload: the layout version its reader checks.
    void Version(uint32_t version)
    {
        U32(version);
    }
    void U8(uint8_t v)
    {
        _bytes.push_back(v);
    }
    void Bool(bool v)
    {
        U8(v ? 1 : 0);
    }
    void U32(uint32_t v)
    {
        for (int i = 0; i < 4; ++i)
            _bytes.push_back(uint8_t(v >> (8 * i)));
    }
    void I32(int32_t v)
    {
        U32(uint32_t(v));
    }
    void U64(uint64_t v)
    {
        for (int i = 0; i < 8; ++i)
            _bytes.push_back(uint8_t(v >> (8 * i)));
    }
    void F32(float v)
    {
        uint32_t bits;
        std::memcpy(&bits, &v, sizeof bits);
        U32(bits);
    }
    void F64(double v)
    {
        uint64_t bits;
        std::memcpy(&bits, &v, sizeof bits);
        U64(bits);
    }
    template <class E> void Enum(E v)
    {
        static_assert(std::is_enum<E>::value, "Enum takes an enum");
        U32(uint32_t(v));
    }
    void Count(size_t n)
    {
        U64(uint64_t(n));
    }
    /// A list: its length, then \p each(writer, item) per item.
    template <class T, class Fn> void List(const std::vector<T> &items, Fn each)
    {
        Count(items.size());
        for (const T &item : items)
            each(*this, item);
    }

private:
    std::vector<uint8_t> &_bytes;
};

/// Reads values a RigExecPayloadWriter wrote. After the first failure every
/// read returns zero and Ok is false.
class RigExecPayloadReader {
public:
    RigExecPayloadReader(const uint8_t *bytes, size_t size)
        : _bytes(bytes), _size(bytes ? size : 0)
    {
    }

    bool Ok() const
    {
        return _ok;
    }
    const std::string &Error() const
    {
        return _error;
    }
    /// Stops reading with \p why; keeps the first reason.
    void Fail(const char *why)
    {
        if (_ok) {
            _ok = false;
            _error = why;
        }
    }

    /// Checks the layout version a writer started with.
    void Version(uint32_t supported)
    {
        const uint32_t version = U32();
        if (_ok && version != supported) {
            Fail("unsupported payload version");
        }
    }
    uint8_t U8()
    {
        if (!_Take(1))
            return 0;
        return _bytes[_at - 1];
    }
    bool Bool()
    {
        const uint8_t v = U8();
        if (v > 1)
            Fail("a flag is neither 0 nor 1");
        return v == 1;
    }
    uint32_t U32()
    {
        if (!_Take(4))
            return 0;
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i)
            v |= uint32_t(_bytes[_at - 4 + i]) << (8 * i);
        return v;
    }
    int32_t I32()
    {
        return int32_t(U32());
    }
    uint64_t U64()
    {
        if (!_Take(8))
            return 0;
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i)
            v |= uint64_t(_bytes[_at - 8 + i]) << (8 * i);
        return v;
    }
    float F32()
    {
        const uint32_t bits = U32();
        float v;
        std::memcpy(&v, &bits, sizeof v);
        return v;
    }
    double F64()
    {
        const uint64_t bits = U64();
        double v;
        std::memcpy(&v, &bits, sizeof v);
        return v;
    }
    /// An enum value no greater than \p last.
    template <class E> E Enum(E last)
    {
        static_assert(std::is_enum<E>::value, "Enum takes an enum");
        const uint32_t v = U32();
        if (v > uint32_t(last)) {
            Fail("an enum value is out of range");
            return E{};
        }
        return E(v);
    }
    /// A list length, refused when the remaining bytes cannot hold that
    /// many items of at least \p minItemBytes each.
    size_t Count(size_t minItemBytes)
    {
        const uint64_t n = U64();
        const size_t per = minItemBytes ? minItemBytes : 1;
        if (_ok && n > (_size - _at) / per) {
            Fail("a list is longer than the bytes left");
            return 0;
        }
        return size_t(n);
    }
    /// A list RigExecPayloadWriter::List wrote, each item read by
    /// \p each(reader, item).
    template <class T, class Fn>
    void List(std::vector<T> &items, size_t minItemBytes, Fn each)
    {
        const size_t n = Count(minItemBytes);
        items.clear();
        if (!_ok)
            return;
        items.resize(n);
        for (T &item : items) {
            each(*this, item);
            if (!_ok)
                return;
        }
    }
    /// True when every byte was read without a failure; otherwise false
    /// with the reason in \p error.
    bool Finish(std::string *error)
    {
        if (_ok && _at != _size)
            Fail("bytes are left after the payload");
        if (!_ok && error)
            *error = _error;
        return _ok;
    }

private:
    bool _Take(size_t n)
    {
        if (!_ok)
            return false;
        if (n > _size - _at) {
            Fail("the payload ends early");
            return false;
        }
        _at += n;
        return true;
    }

    const uint8_t *_bytes;
    size_t _size;
    size_t _at = 0;
    bool _ok = true;
    std::string _error;
};

} // namespace rigExec

#endif
