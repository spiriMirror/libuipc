#pragma once
#include <muda/muda_def.h>
// Skip if the backend's type_define.h already provided the global ::arg shim
// (UIPC_EIGEN_ARG_SHIM), to avoid a duplicate definition.
#if defined(__CUDA_ARCH__) && !defined(UIPC_EIGEN_ARG_SHIM)
#include <complex>
// Fix eigen cuda cxx20 : can't find `arg` in global scope
template <typename T>
MUDA_INLINE MUDA_GENERIC T arg(const std::complex<T>& z)
{
    return std::atan2(std::imag(z), std::real(z));
}
#endif
