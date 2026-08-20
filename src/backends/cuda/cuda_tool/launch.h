#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/view.h>
#include <unordered_map>

namespace uipc::backend::cuda_tool
{
// Best occupancy block size for a kernel (cached per kernel symbol), matching
// what the old ParallelFor's automatic selection would pick. Named-kernel
// launch sites use this to keep the same FP behavior as the lambda
// ParallelFor they replaced.
//
// The cache is keyed by kernel address: distinct kernels may share the same
// function-pointer type (same signature), so keying by the template
// parameter would let one kernel's block size leak into another's.
template <typename Kernel>
int best_block_dim(Kernel kernel, size_t shared_mem_size = 0)
{
    static thread_local std::unordered_map<const void*, int> cached_block_sizes;
    const void* key = (const void*)kernel;
    auto        it  = cached_block_sizes.find(key);
    if(it == cached_block_sizes.end())
    {
        int min_grid_size = -1;
        int block_size    = -1;
        CUDA_TOOL_CHECK(cudaOccupancyMaxPotentialBlockSize(
            &min_grid_size, &block_size, kernel, shared_mem_size));
        it = cached_block_sizes.emplace(key, block_size).first;
    }
    return it->second;
}

// grid dim for launching `kernel` over n items with best_block_dim(kernel)
template <typename Kernel>
int best_grid_dim(int n, Kernel kernel, size_t shared_mem_size = 0)
{
    int bd = best_block_dim(kernel, shared_mem_size);
    return (n + bd - 1) / bd;
}
}  // namespace uipc::backend::cuda_tool
