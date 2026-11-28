#ifndef RIGEXEC_MATH_BORROWED_SPAN_H
#define RIGEXEC_MATH_BORROWED_SPAN_H

#include <cstddef>
#include <vector>

namespace rigExec {

/// A run of values another owner holds, read like the std::vector a kernel
/// setting otherwise owns: valid while that owner keeps the values.
template <class T> struct RigExecBorrowedSpan {
    const T *items = nullptr;
    size_t count = 0;

    RigExecBorrowedSpan() = default;
    RigExecBorrowedSpan(const T *data, size_t size) : items(data), count(size)
    {
    }
    RigExecBorrowedSpan(const std::vector<T> &values)
        : items(values.data()), count(values.size())
    {
    }
    size_t size() const
    {
        return count;
    }
    bool empty() const
    {
        return count == 0;
    }
    const T *data() const
    {
        return items;
    }
    const T &operator[](size_t i) const
    {
        return items[i];
    }
    const T *begin() const
    {
        return items;
    }
    const T *end() const
    {
        return items + count;
    }
};

/// Equal when both hold the same values, compared as std::vector compares.
template <class T>
bool
operator==(const RigExecBorrowedSpan<T> &a, const RigExecBorrowedSpan<T> &b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (!(a[i] == b[i]))
            return false;
    return true;
}

template <class T>
bool
operator!=(const RigExecBorrowedSpan<T> &a, const RigExecBorrowedSpan<T> &b)
{
    return !(a == b);
}

} // namespace rigExec

#endif
