#pragma once

#include <array>
#include <concepts>
#include <cstddef>

namespace ELN
{
// Naming convention for utility functions (mimicking std convention): function_name()
namespace Utils
{
template <std::integral T>
constexpr T int_pow(T base, T exp)
{
    T res {1};
    for (T power {1}; power <= exp; ++power)
        res *= base;
    return res;
}

template <size_t N, typename T>
constexpr auto array_fill(const T &value)
{
    std::array<T, N> array {};
    array.fill(value);
    return array;
}

template <size_t N, std::integral T>
constexpr auto array_iota(T start)
{
    std::array<T, N> array {};
    for (T i {0}; i < N; ++i)
        array[i] = start + i;
    return array;
}

} // namespace Utils
} // namespace ELN
