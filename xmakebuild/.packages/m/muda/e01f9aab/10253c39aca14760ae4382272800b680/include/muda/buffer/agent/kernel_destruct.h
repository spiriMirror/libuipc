#pragma once
#include <cuda.h>
#include <muda/buffer/buffer_fwd.h>


namespace muda::details::buffer
{
// destruct 0D
template <typename T>
MUDA_HOST void kernel_destruct(cudaStream_t stream, VarView<T> view);

// destruct 1D
template <typename T>
MUDA_HOST void kernel_destruct(int grid_dim, int block_dim, cudaStream_t stream, BufferView<T> buffer_view);

// destruct 2D
template <typename T>
MUDA_HOST void kernel_destruct(int             grid_dim,
                               int             block_dim,
                               cudaStream_t    stream,
                               Buffer2DView<T> buffer_view);

// destruct 3D
template <typename T>
MUDA_HOST void kernel_destruct(int             grid_dim,
                               int             block_dim,
                               cudaStream_t    stream,
                               Buffer3DView<T> buffer_view);
}  // namespace muda::details::buffer

#include "details/kernel_destruct.inl"