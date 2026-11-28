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
#include <vector>

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
        if (size && data)
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
                if (other._size && other._data)
                    std::memcpy(_data, other._data, other._size * sizeof(T));
                _size = other._size;
            }
        }
        return *this;
    }
    /// An array reading the \p size elements at \p data, which stay the
    /// caller's and unchanged while it is read. Resizing or assigning it
    /// copies them into storage of its own first; copying it copies them.
    static RigExecArray Borrow(const T *data, size_t size)
    {
        RigExecArray view;
        view._data = const_cast<T *>(data);
        view._size = size;
        return view;
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
        Reserve(size);
        if (size > _size)
            std::fill(_data + _size, _data + size, T());
        _size = size;
    }

    /// Makes room for \p capacity elements, keeping the ones it holds, so
    /// growing to that size later allocates nothing.
    void Reserve(size_t capacity)
    {
        if (capacity <= _capacity)
            return;
        RigExecArray grown;
        grown._Allocate(capacity);
        if (_size && _data)
            std::memcpy(grown._data, _data, _size * sizeof(T));
        grown._size = _size;
        _Take(grown);
    }

    /// Holds \p size value-initialized elements, in the storage it has
    /// when that is large enough.
    void Assign(size_t size)
    {
        Reserve(size);
        std::fill(_data, _data + size, T());
        _size = size;
    }

    /// Holds a copy of \p size elements from \p data, in the storage it
    /// has when that is large enough.
    void Assign(const T *data, size_t size)
    {
        Reserve(size);
        if (size)
            std::memcpy(_data, data, size * sizeof(T));
        _size = size;
    }

    size_t Capacity() const
    {
        return _capacity;
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

/// A list of arrays, each read or held as a RigExecArray: the storage of
/// the standard array-list types. Copying copies every element's values.
template <class T> class RigExecArrayList {
public:
    size_t Size() const
    {
        return _items.size();
    }
    void Resize(size_t size)
    {
        _items.resize(size);
    }
    void Reserve(size_t capacity)
    {
        _items.reserve(capacity);
    }
    RigExecArray<T> *Data()
    {
        return _items.data();
    }
    const RigExecArray<T> *Data() const
    {
        return _items.data();
    }
    RigExecArray<T> &operator[](size_t i)
    {
        return _items[i];
    }
    const RigExecArray<T> &operator[](size_t i) const
    {
        return _items[i];
    }

private:
    std::vector<RigExecArray<T>> _items;
};

} // namespace rigExecStd

#endif
