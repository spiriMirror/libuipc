#pragma once
#include <cinttypes>
namespace muda
{
#ifdef _WIN32
constexpr size_t operator"" _K(size_t value)
{
    return value * 1024;
}

constexpr size_t operator"" _M(size_t value)
{
    return value * 1024 * 1024;
}

constexpr size_t operator"" _G(size_t value)
{
    return value * 1024 * 1024 * 1024;
}

constexpr size_t operator"" _T(size_t value)
{
    return value * 1024 * 1024 * 1024 * 1024;
}

constexpr size_t operator"" _P(size_t value)
{
    return value * 1024 * 1024 * 1024 * 1024 * 1024;
}
#elif __linux__
// TOFIX:
// temporary workaround for linux
constexpr size_t operator"" _K(unsigned long long value)
{
    return value * 1024;
}

constexpr size_t operator"" _M(unsigned long long value)
{
    return value * 1024 * 1024;
}

constexpr size_t operator"" _G(unsigned long long value)
{
    return value * 1024 * 1024 * 1024;
}

constexpr size_t operator"" _T(unsigned long long value)
{
    return value * 1024 * 1024 * 1024 * 1024;
}

constexpr size_t operator"" _P(unsigned long long value)
{
    return value * 1024 * 1024 * 1024 * 1024 * 1024;
}
#endif
}  // namespace muda