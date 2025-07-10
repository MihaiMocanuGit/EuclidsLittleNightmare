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
constexpr auto fill_array(const T &value)
{
    std::array<T, N> array {};
    array.fill(value);
    return array;
}

} // namespace Utils
} // namespace ELN
