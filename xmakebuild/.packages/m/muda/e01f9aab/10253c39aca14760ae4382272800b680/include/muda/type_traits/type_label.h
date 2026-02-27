#pragma once
#include <type_traits>

namespace muda
{
/*************************************************************************
* 
*                               Type Labels
* 
*************************************************************************/

// force a type to be trivial (only in muda memory-related API)
template <typename T>
struct force_trivial
{
    constexpr static bool value = false;
};
template <typename T>
constexpr bool force_trivial_v = force_trivial<T>::value;
template <typename T>
constexpr bool is_trivial_v = std::is_trivial_v<T> || force_trivial_v<T>;

// force a type to be trivially destructible (only in muda memory-related API)
template <typename T>
struct force_trivially_destructible
{
    constexpr static bool value = false;
};
template <typename T>
constexpr bool force_trivially_destructible_v = force_trivially_destructible<T>::value;
template <typename T>
constexpr bool is_trivially_destructible_v =
    std::is_trivially_destructible_v<T> || force_trivially_destructible_v<T>;

// force a type to be trivially constructible (only in muda memory-related API)
template <typename T>
struct force_trivially_constructible
{
    constexpr static bool value = false;
};
template <typename T>
constexpr bool force_trivially_constructible_v = force_trivially_constructible<T>::value;
template <typename T>
constexpr bool is_trivially_constructible_v =
    std::is_trivially_constructible_v<T> || force_trivially_constructible_v<T>;

// force a type to be trivially copy constructible (only in muda memory-related API)
template <typename T>
struct force_trivially_copy_constructible
{
    constexpr static bool value = false;
};
template <typename T>
constexpr bool force_trivially_copy_constructible_v =
    force_trivially_copy_constructible<T>::value;
template <typename T>
constexpr bool is_trivially_copy_constructible_v =
    std::is_trivially_copy_constructible_v<T> || force_trivially_copy_constructible_v<T>;

// force a type to be trivially copy assignable (only in muda memory-related API)
template <typename T>
struct force_trivially_copy_assignable
{
    constexpr static bool value = false;
};
template <typename T>
constexpr bool force_trivially_copy_assignable_v =
    force_trivially_copy_assignable<T>::value;
template <typename T>
constexpr bool is_trivially_copy_assignable_v =
    std::is_trivially_copy_assignable_v<T> || force_trivially_copy_assignable_v<T>;
}  // namespace muda