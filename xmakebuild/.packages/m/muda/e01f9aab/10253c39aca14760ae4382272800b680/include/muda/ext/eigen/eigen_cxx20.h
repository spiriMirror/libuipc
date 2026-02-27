#pragma once
#include <muda/muda_def.h>
#ifdef __CUDA_ARCH__
#include <complex>
// Fix eigen cuda cxx20 : can't find `arg` in global scope
template <typename T>
MUDA_INLINE MUDA_GENERIC T arg(const std::complex<T>& z)
{
    return std::atan2(std::imag(z), std::real(z));
}
#endif
