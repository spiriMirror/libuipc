#pragma once
#include <cuda_runtime.h>
// Provide a global-scope ::arg(std::complex) BEFORE any Eigen include. nvcc's
// passes cannot resolve Eigen MathFunctions.h's `using ::arg` otherwise. This must
// precede <uipc/common/type_define.h> (which pulls in <Eigen/Core>).
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
#include <stdexcept>
#include <string>
#include <cstdio>
#include <span>

namespace uipc::backend::cuda_tool
{
using std::span;

// host-side error string without pulling in fmt (avoids nvcc/UTF-8 issues)
inline std::string cuda_error_string(cudaError_t e, const char* expr, const char* file, int line)
{
    char buf[512];
    std::snprintf(buf,
                  sizeof(buf),
                  "[%s:%d] CUDA error %s (%d): %s",
                  file,
                  line,
                  expr,
                  static_cast<int>(e),
                  cudaGetErrorString(e));
    return buf;
}
}  // namespace uipc::backend::cuda_tool

namespace uipc::backend::cuda_tool
{
// Default stream used by all cuda_tool primitives when no stream is given.
// Kept identical to muda's default (the default stream) so behavior is unchanged.
inline cudaStream_t default_stream()
{
    return cudaStream_t{nullptr};
}

inline void check_cuda_error(cudaError_t e, const char* expr, const char* file, int line)
{
    if(e != cudaSuccess)
    {
        throw std::runtime_error{cuda_error_string(e, expr, file, line)};
    }
}

#define CUDA_TOOL_CHECK(EXPR) \
    ::uipc::backend::cuda_tool::check_cuda_error((EXPR), #EXPR, __FILE__, __LINE__)

// Block size used by cuda_tool launches (matches muda's default_parallel_dim).
inline constexpr int default_block_dim = 256;

// Lightweight stream handle wrapper (replacement for cuda_tool::Stream).
// The default stream is a process-wide singleton identical to cudaStream_t{nullptr}.
class Stream
{
  public:
    Stream() = default;
    explicit Stream(cudaStream_t h) noexcept
        : m_handle(h)
    {
    }
    operator cudaStream_t() const noexcept { return m_handle; }
    cudaStream_t view() const noexcept { return m_handle; }

    static Stream& Default() noexcept
    {
        static Stream s{nullptr};
        return s;
    }

  private:
    cudaStream_t m_handle = nullptr;
};
}  // namespace uipc::backend::cuda_tool
