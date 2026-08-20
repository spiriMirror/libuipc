#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/view.h>

namespace uipc::backend::cuda_tool
{
// Best occupancy block size for a kernel (cached per kernel symbol), matching
// what the old ParallelFor's automatic selection would pick. Named-kernel
// launch sites use this to keep the same FP behavior as the lambda
// ParallelFor they replaced.
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
