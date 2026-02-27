#include <muda/type_traits/type_label.h>
#include <muda/launch/parallel_for.h>
#include <muda/buffer/buffer_view.h>
#include <muda/buffer/buffer_2d_view.h>
#include <muda/buffer/buffer_3d_view.h>

namespace muda::details::buffer
{
// assign 0D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_assign(cudaStream_t stream, VarView<T> dst, CVarView<T> src)
{
    ParallelFor(1, 1, 0, stream)
        .apply(1,
               [dst, src] __device__(int i) mutable
               { *dst.data() = *src.data(); });
}

// assign 1D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_assign(int            grid_dim,
                                         int            block_dim,
                                         cudaStream_t   stream,
                                         BufferView<T>  dst,
                                         CBufferView<T> src)
{
    ParallelFor(grid_dim, block_dim, 0, stream)
        .apply(dst.size(),
               [dst, src] __device__(int i) mutable
               { *dst.data(i) = *src.data(i); });
}

// assign 2D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_assign(int              grid_dim,
                                         int              block_dim,
                                         cudaStream_t     stream,
                                         Buffer2DView<T>  dst,
                                         CBuffer2DView<T> src)
{
    ParallelFor(grid_dim, block_dim, 0, stream)
        .apply(dst.total_size(),
               [dst, src] __device__(int i) mutable
               { *dst.data(i) = *src.data(i); });
}

// assign 3D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_assign(int              grid_dim,
                                         int              block_dim,
                                         cudaStream_t     stream,
                                         Buffer3DView<T>  dst,
                                         CBuffer3DView<T> src)
{
    ParallelFor(grid_dim, block_dim, 0, stream)
        .apply(dst.total_size(),
               [dst, src] __device__(int i) mutable
               { *dst.data(i) = *src.data(i); });
}
}  // namespace muda::details::buffer