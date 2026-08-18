#pragma once
/********************************************************************
 * @file   type_define.h
 * @brief  Launch-qualifier macros and Eigen triviality traits for the CUDA backend.
 *
 * Kept lightweight on purpose: this header is pulled into host (.cpp) translation
 * units, so it must NOT include the heavy device headers (muda.h / launch kernels).
 * Only the muda_def qualifiers and the Eigen triviality traits are needed here.
 *********************************************************************/
#include <muda/muda_def.h>
#include <muda/type_traits/type_label.h>
// Eigen's `using ::arg` for std::complex needs a global-scope overload in BOTH of
// nvcc's passes (the template is instantiated on host too). Provide it once via an
// include guard so it doesn't collide with the vendored muda eigen_cxx20.h shim.
#if defined(__CUDACC__) && !defined(UIPC_EIGEN_ARG_SHIM)
#define UIPC_EIGEN_ARG_SHIM
#include <complex>
template <typename T>
__host__ __device__ inline T arg(const std::complex<T>& z)
{
    return std::atan2(std::imag(z), std::real(z));
}
#endif
#include <uipc/common/type_define.h>

#define UIPC_GENERIC MUDA_GENERIC
#define UIPC_DEVICE MUDA_DEVICE
#define UIPC_HOST MUDA_HOST

#if __INTELLISENSE__
// Just for Visual Studio IntelliSense: NVCC failed to define the UIPC_RELATIVE_SOURCE_FILE
#define UIPC_RELATIVE_SOURCE_FILE "rel_path_of(" __FILE__ ")"
#endif
