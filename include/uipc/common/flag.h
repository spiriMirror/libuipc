#pragma once
#include <concepts>
#include <type_traits>
#include <magic_enum/magic_enum.hpp>
#include <magic_enum/magic_enum_flags.hpp>

namespace uipc
{
template <typename T>
    requires std::is_enum_v<T>
constexpr auto to_underlying(const T& e) noexcept
{
    return static_cast<std::underlying_type_t<T>>(e);
}

template <typename T>
    requires std::is_enum_v<T>
constexpr bool has_flags(const T& flags, const T& test_flags) noexcept
{
    using namespace magic_enum::bitwise_operators;
    return (flags & test_flags) == test_flags;
}

/**
 * @brief Check if the given flag is a valid flag (only one bit is set). 
 */
template <typename T>
    requires std::is_enum_v<T>
constexpr bool is_valid_flag(const T& flag) noexcept
{
    auto value = to_underlying(flag);
    return value != 0 && (value & (value - 1)) == 0;
}
}  // namespace uipc