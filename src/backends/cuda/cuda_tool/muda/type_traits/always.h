#pragma once

namespace muda
{
template <typename T>
struct always_false
{
    static constexpr bool value = false;
};

template <typename T>
constexpr bool always_false_v = always_false<T>::value;

template <typename T>
struct always_true
{
    static constexpr bool value = true;
};
}  // namespace muda