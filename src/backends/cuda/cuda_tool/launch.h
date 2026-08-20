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
    void launch_parallel_for(int n, F&& f, int grid_dim, int block_dim, cudaStream_t stream,
                             size_t shared_mem_size = 0)
    {
        if(n <= 0)
            return;
        using CallableType = std::decay_t<F>;
        int bd             = block_dim;
        if(bd <= 0)
        {
            // automatic choose (muda parity): best occupancy block size per kernel
            static thread_local int cached_block_size = -1;
            if(cached_block_size <= 0)
            {
                int min_grid_size = -1;
                CUDA_TOOL_CHECK(cudaOccupancyMaxPotentialBlockSize(
                    &min_grid_size,
                    &cached_block_size,
                    parallel_for_kernel<CallableType>,
                    shared_mem_size));
            }
            bd = cached_block_size;
        }
        if(grid_dim > 0)
        {
            // explicit grid: grid-stride loop (muda parity)
            parallel_for_gs_kernel<CallableType>
                <<<grid_dim, bd, shared_mem_size, stream>>>(n, std::forward<F>(f));
        }
        else
        {
            int gd = (n + bd - 1) / bd;
            parallel_for_kernel<CallableType>
                <<<gd, bd, shared_mem_size, stream>>>(n, std::forward<F>(f));
        }
    }
}  // namespace details

// A minimal, fluent replacement for cuda_tool::ParallelFor (same constructor set).
//
//   ParallelFor().apply(n, kernel);        // default 256-thread blocks
//   ParallelFor(64).apply(n, kernel);      // 64-thread blocks
//   ParallelFor(2, 64).apply(n, kernel);   // 2 blocks x 64 threads
class ParallelFor
{
  public:
    ParallelFor(size_t shared_mem_size = 0, cudaStream_t stream = nullptr)
        : m_stream(stream)
        , m_shared_mem_size(shared_mem_size)
    {
    }
    ParallelFor(int block_dim, size_t shared_mem_size, cudaStream_t stream = nullptr)
        : m_stream(stream)
        , m_block_dim(block_dim)
        , m_shared_mem_size(shared_mem_size)
    {
    }
    ParallelFor(int          grid_dim,
                int          block_dim,
                size_t       shared_mem_size = 0,
                cudaStream_t stream          = nullptr)
        : m_stream(stream)
        , m_grid_dim(grid_dim)
        , m_block_dim(block_dim)
        , m_shared_mem_size(shared_mem_size)
    {
    }

    // optional launch metadata (kept for parity with muda; currently a no-op tag)
    ParallelFor& kernel_name(std::string_view) { return *this; }
    ParallelFor& file_line(std::string_view, int) { return *this; }

    template <typename F>
    ParallelFor& apply(int n, F&& f) &
    {
        details::launch_parallel_for(n,
                                     std::forward<F>(f),
                                     m_grid_dim,
                                     m_block_dim,
                                     m_stream,
                                     m_shared_mem_size);
        return *this;
    }
    template <typename F>
    ParallelFor& apply(int n, F&& f) &&
    {
        details::launch_parallel_for(n,
                                     std::forward<F>(f),
                                     m_grid_dim,
                                     m_block_dim,
                                     m_stream,
                                     m_shared_mem_size);
        return *this;
    }

    ParallelFor& wait()
    {
        CUDA_TOOL_CHECK(cudaStreamSynchronize(m_stream));
        return *this;
    }

  private:
    cudaStream_t m_stream          = default_stream();
    int          m_grid_dim        = 0;
    int          m_block_dim       = -1;  // <=0: auto-choose by occupancy (muda parity)
    size_t       m_shared_mem_size = 0;
};

// Best occupancy block size for a kernel (cached per kernel symbol), matching
// what ParallelFor's automatic selection would pick. Named-kernel launch sites
// use this to keep the same FP behavior as the lambda ParallelFor they replace.
template <typename Kernel>
int best_block_dim(Kernel kernel, size_t shared_mem_size = 0)
{
    static thread_local int cached_block_size = -1;
    if(cached_block_size <= 0)
    {
        int min_grid_size = -1;
        CUDA_TOOL_CHECK(cudaOccupancyMaxPotentialBlockSize(
            &min_grid_size, &cached_block_size, kernel, shared_mem_size));
    }
    return cached_block_size;
}

// grid dim for launching `kernel` over n items with best_block_dim(kernel)
template <typename Kernel>
int best_grid_dim(int n, Kernel kernel, size_t shared_mem_size = 0)
{
    int bd = best_block_dim(kernel, shared_mem_size);
    return (n + bd - 1) / bd;
}
}  // namespace uipc::backend::cuda_tool
