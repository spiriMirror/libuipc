#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/view.h>

namespace uipc::backend::cuda_tool
{
namespace details
{
    // generic N-body kernel: F(int i)
    template <typename F>
    __global__ void parallel_for_kernel(int n, F f)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        f(i);
    }

    // grid-stride variant used for indeterminate / large workloads
    template <typename F>
    __global__ void parallel_for_gs_kernel(int n, F f)
    {
        for(int i = blockIdx.x * blockDim.x + threadIdx.x; i < n;
            i += gridDim.x * blockDim.x)
            f(i);
    }

    template <typename F>
    void launch_parallel_for(int n, F&& f, int grid_dim, int block_dim, cudaStream_t stream)
    {
        if(n <= 0)
            return;
        if(grid_dim <= 0)
            grid_dim = (n + block_dim - 1) / block_dim;
        parallel_for_kernel<<<grid_dim, block_dim, 0, stream>>>(n, std::forward<F>(f));
    }
}  // namespace details

// A minimal, fluent replacement for muda::ParallelFor.
//
//   parallel_for().apply(n, [view = buffer.viewer()] __device__(int i) { ... });
//   parallel_for(grid, block).apply(n, [] __device__(int i) { ... });
class ParallelFor
{
  public:
    ParallelFor() = default;
    explicit ParallelFor(cudaStream_t stream)
        : m_stream(stream)
    {
    }
    ParallelFor(int grid_dim, int block_dim)
        : m_grid_dim(grid_dim)
        , m_block_dim(block_dim)
    {
    }
    ParallelFor(cudaStream_t stream, int grid_dim, int block_dim)
        : m_stream(stream)
        , m_grid_dim(grid_dim)
        , m_block_dim(block_dim)
    {
    }

    // optional launch metadata (kept for parity with muda; currently a no-op tag)
    ParallelFor& kernel_name(std::string_view) { return *this; }
    ParallelFor& file_line(std::string_view, int) { return *this; }

    template <typename F>
    ParallelFor& apply(int n, F&& f) &
    {
        details::launch_parallel_for(
            n, std::forward<F>(f), m_grid_dim, m_block_dim, m_stream);
        return *this;
    }
    template <typename F>
    ParallelFor& apply(int n, F&& f) &&
    {
        details::launch_parallel_for(
            n, std::forward<F>(f), m_grid_dim, m_block_dim, m_stream);
        return *this;
    }

    ParallelFor& wait()
    {
        CUDA_TOOL_CHECK(cudaStreamSynchronize(m_stream));
        return *this;
    }

  private:
    cudaStream_t m_stream    = default_stream();
    int          m_grid_dim  = 0;
    int          m_block_dim = default_block_dim;
};

// free-function convenience
template <typename F>
void parallel_for(int n, F&& f, cudaStream_t stream = default_stream())
{
    details::launch_parallel_for(n, std::forward<F>(f), 0, default_block_dim, stream);
}

// Simple kernel launch helper for raw kernels.
struct Launch
{
    cudaStream_t stream    = default_stream();
    int          grid_dim  = 0;
    int          block_dim = default_block_dim;

    Launch() = default;
    Launch(int grid, int block, cudaStream_t s = default_stream())
        : stream(s)
        , grid_dim(grid)
        , block_dim(block)
    {
    }
    Launch& file_line(std::string_view, int) { return *this; }
    Launch& kernel_name(std::string_view) { return *this; }

    template <typename F>
    Launch& apply(F&& f)
    {
        details::launch_parallel_for(
            0, std::forward<F>(f), grid_dim, block_dim, stream);
        return *this;
    }
    Launch& wait()
    {
        CUDA_TOOL_CHECK(cudaStreamSynchronize(stream));
        return *this;
    }
};
}  // namespace uipc::backend::cuda_tool
