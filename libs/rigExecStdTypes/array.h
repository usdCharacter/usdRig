#ifndef RIGEXEC_STD_TYPES_ARRAY_H
#define RIGEXEC_STD_TYPES_ARRAY_H

/// An owned array of a trivially copyable element, the storage of every
/// standard array type. Elements start at 16 bytes, or 256 from 4 KiB, so SIMD
/// loops and device copies need no realignment.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <type_traits>

namespace rigExecStd {

template <class T> class RigExecArray {
    static_assert(std::is_trivially_copyable_v<T>,
                  "RigExecArray holds trivially copyable elements only");

public:
    RigExecArray() = default;
    explicit RigExecArray(size_t size)
    {
        Resize(size);
    }
    RigExecArray(const T *data, size_t size)
    {
        _Allocate(size);
        if (size)
            std::memcpy(_data, data, size * sizeof(T));
    }
    RigExecArray(const RigExecArray &other)
        : RigExecArray(other._data, other._size)
    {
    }
    RigExecArray(RigExecArray &&other) noexcept
    {
        _Take(other);
    }
    RigExecArray &operator=(const RigExecArray &other)
    {
        if (this != &other) {
            if (other._size > _capacity) {
                RigExecArray copy(other);
                _Take(copy);
            } else {
                if (other._size)
                    std::memcpy(_data, other._data, other._size * sizeof(T));
                _size = other._size;
            }
        }
        return *this;
    }
    RigExecArray &operator=(RigExecArray &&other) noexcept
    {
        if (this != &other)
            _Take(other);
        return *this;
    }

    /// Keeps the first min(size, Size()) elements; new ones are
    /// value-initialized.
    void Resize(size_t size)
    {
        if (size > _capacity) {
            RigExecArray grown;
            grown._Allocate(size);
            if (_size)
                std::memcpy(grown._data, _data, _size * sizeof(T));
            _Take(grown);
        }
        if (size > _size)
            std::fill(_data + _size, _data + size, T());
        _size = size;
    }

    size_t Size() const
    {
        return _size;
    }
    bool Empty() const
    {
        return _size == 0;
    }
    T *Data()
    {
        return _data;
    }
    const T *Data() const
    {
        return _data;
    }
    T &operator[](size_t i)
    {
        return _data[i];
    }
    const T &operator[](size_t i) const
    {
        return _data[i];
    }
    T *begin()
    {
        return _data;
    }
    T *end()
    {
        return _data + _size;
    }
    const T *begin() const
    {
        return _data;
    }
    const T *end() const
    {
        return _data + _size;
    }

    /// Start alignment of an array of \p size elements.
    static constexpr size_t AlignmentFor(size_t size)
    {
        return size * sizeof(T) >= 4096 ? 256 : 16;
    }

private:
    void _Allocate(size_t size)
    {
        const size_t align = AlignmentFor(size);
        _storage = std::make_unique<std::byte[]>(size * sizeof(T) + align);
        const uintptr_t raw = reinterpret_cast<uintptr_t>(_storage.get());
        _data = reinterpret_cast<T *>(_storage.get() +
                                      (align - raw % align) % align);
        _capacity = size;
        _size = size;
        if (size)
            std::fill(_data, _data + size, T());
    }

    void _Take(RigExecArray &other)
    {
        _storage = std::move(other._storage);
        _data = other._data;
        _size = other._size;
        _capacity = other._capacity;
        other._data = nullptr;
        other._size = 0;
        other._capacity = 0;
    }

    std::unique_ptr<std::byte[]> _storage;
    T *_data = nullptr;
    size_t _size = 0;
    size_t _capacity = 0;
};

} // namespace rigExecStd

#endif
