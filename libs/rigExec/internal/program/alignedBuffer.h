#ifndef RIGEXEC_INTERNAL_PROGRAM_ALIGNED_BUFFER_H
#define RIGEXEC_INTERNAL_PROGRAM_ALIGNED_BUFFER_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

namespace rigExec {

/// Zeroed bytes whose start is aligned to kAlign.
class RigExecAlignedBuffer {
public:
    static constexpr size_t kAlign = 256;

    void Allocate(size_t size)
    {
        _size = size;
        _storage = std::make_unique<std::byte[]>(size + kAlign);
        const uintptr_t raw = reinterpret_cast<uintptr_t>(_storage.get());
        _data = _storage.get() + ((kAlign - raw % kAlign) % kAlign);
        std::memset(_data, 0, size);
    }
    std::byte *Data() const
    {
        return _data;
    }
    size_t Size() const
    {
        return _size;
    }

private:
    std::unique_ptr<std::byte[]> _storage;
    std::byte *_data = nullptr;
    size_t _size = 0;
};

/// Offset of the next value of \p size and \p align at or after \p offset.
/// Values from 4 KiB start on 256 bytes, others on at least 16.
inline size_t
RigExecPlaceValue(size_t offset, size_t size, size_t align)
{
    size_t a = align < 16 ? 16 : align;
    if (size >= 4096 && a < 256)
        a = 256;
    return (offset + a - 1) / a * a;
}

} // namespace rigExec

#endif
