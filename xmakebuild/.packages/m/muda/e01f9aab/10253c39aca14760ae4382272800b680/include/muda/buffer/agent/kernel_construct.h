#pragma once
#include <cuda.h>
#include <muda/buffer/buffer_fwd.h>


namespace muda::details::buffer
{
// construct 0D
template <typename T>
MUDA_HOST void kernel_construct(cudaStream_t stream, VarView<T> view);

// construct 1D
template <typename T>
MUDA_HOST void kernel_construct(int           grid_dim,
                                int           block_dim,
                                cudaStream_t  stream,
                                BufferView<T> buffer_view);

// construct 2D
template <typename T>
MUDA_HOST void kernel_construct(int             grid_dim,
                                int             block_dim,
                                cudaStream_t    stream,
                                Buffer2DView<T> buffer_view);

// construct 3D
template <typename T>
MUDA_HOST void kernel_construct(int             grid_dim,
                                int             block_dim,
                                cudaStream_t    stream,
                                Buffer3DView<T> buffer_view);
}  // namespace muda::details::buffer

#include "details/kernel_construct.inl"