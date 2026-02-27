#pragma once
#include <muda/muda_def.h>
#include <array>
#include <bitset>
namespace muda::details::buffer
{
MUDA_INLINE void swap(bool b, size_t& l, size_t& r) MUDA_NOEXCEPT
{
    if(b)
    {
        size_t tmp = l;
        l          = r;
        r          = tmp;
    }
}

template <size_t N>
MUDA_INLINE void swap(std::bitset<N>         mask,
                      std::array<size_t, N>& lhs,
                      std::array<size_t, N>& rhs) MUDA_NOEXCEPT
{
#pragma unroll
    for(size_t i = 0; i < N; ++i)
        swap(mask[i], lhs[i], rhs[i]);
}
}  // namespace muda::details::buffer