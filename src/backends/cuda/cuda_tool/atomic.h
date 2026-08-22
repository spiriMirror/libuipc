#pragma once
// Scalar atomic wrappers (replacement for cuda_tool::atomic_add / cuda_tool::atomic_exch).
#include <cuda_tool/stream.h>

namespace uipc::backend::cuda_tool
{
// returns the value at `addr` BEFORE the add (CUDA atomicAdd semantics)
template <typename T>
__device__ inline T atomic_add(T* addr, T val)
{
    return ::atomicAdd(addr, val);
}

// returns the value at `addr` BEFORE the exchange
template <typename T>
__device__ inline T atomic_exch(T* addr, T val)
{
    return ::atomicExch(addr, val);
}
}  // namespace uipc::backend::cuda_tool
