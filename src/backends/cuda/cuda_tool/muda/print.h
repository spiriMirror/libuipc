#pragma once
#include <cstdio>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

namespace muda
{
template <typename InType, typename OutType = InType>
MUDA_INLINE MUDA_GENERIC OutType print_convert(const InType& v)
{
    return v;
}

MUDA_INLINE MUDA_GENERIC auto print_convert(const char* v)
{
    return v;
}

template <typename T>
MUDA_INLINE MUDA_GENERIC const T& print_check(const T& t)
{
    static_assert(std::is_arithmetic_v<T> || std::is_pointer_v<T>
                      || std::is_same_v<T, std::nullptr_t>,
                  "not supported type T in printf!");
    return t;
}

MUDA_INLINE MUDA_GENERIC auto print_check(const char* t)
{
    return t;
}

template <typename... Args>
MUDA_INLINE MUDA_GENERIC void print(const char* const fmt, Args&&... arg)
{
    ::printf(fmt, print_check(print_convert(std::forward<Args>(arg)))...);
}
}  // namespace muda
