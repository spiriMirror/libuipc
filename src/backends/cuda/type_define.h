#pragma once
/********************************************************************
 * @file   type_define.h
 * @brief  Launch-qualifier macros and Eigen triviality traits for the CUDA backend.
 *
 * Kept lightweight on purpose: this header is pulled into host (.cpp) translation
 * units, so it must NOT include the heavy device headers (launch kernels).
 * Qualifier macros mirror the semantics the backend always compiled with:
 * empty in pure host TUs, `__host__ __device__` under nvcc's device pass.
 *********************************************************************/
// Eigen's `using ::arg` for std::complex needs a global-scope overload in BOTH of
// nvcc's passes (the template is instantiated on host too). Provide it once via an
// include guard so it doesn't collide with any other shim.
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

#define UIPC_HOST __host__
#define UIPC_DEVICE __device__
#define UIPC_GLOBAL __global__
#define UIPC_SHARED __shared__

#ifdef __CUDA_ARCH__
#define UIPC_GENERIC __host__ __device__
#else
#define UIPC_GENERIC
#endif

#define UIPC_INLINE inline

#if __INTELLISENSE__
// Just for Visual Studio IntelliSense: NVCC failed to define the UIPC_RELATIVE_SOURCE_FILE
#define UIPC_RELATIVE_SOURCE_FILE "rel_path_of(" __FILE__ ")"
#endif
