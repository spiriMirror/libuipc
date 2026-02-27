#pragma once
#include <muda/muda_def.h>
#include <muda/tools/debug_break.h>

namespace muda
{
MUDA_INLINE MUDA_GENERIC void trap() MUDA_NOEXCEPT
{
#ifdef __CUDA_ARCH__
    __trap();
#else
    std::abort();
#endif
}

MUDA_INLINE MUDA_GENERIC void brkpt() MUDA_NOEXCEPT
{
#ifdef __CUDA_ARCH__
    __brkpt();
#else
    debug_break();
#endif
}
}  // namespace muda