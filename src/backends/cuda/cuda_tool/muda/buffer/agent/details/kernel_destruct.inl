#include <muda/type_traits/type_label.h>
#include <muda/launch/parallel_for.h>
#include <muda/buffer/buffer_view.h>
#include <muda/buffer/buffer_2d_view.h>
#include <muda/buffer/buffer_3d_view.h>

namespace muda::details::buffer
{
// destruct 0D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_destruct(cudaStream_t stream, VarView<T> view)
{
    // no need to destruct trivially destructible types
    if constexpr(muda::is_trivially_destructible_v<T>)
        return;

    ParallelFor(1, 1, 0, stream)
        .apply(1, [view] __device__(int i) mutable { view.data()->~T(); });
}

// destruct 1D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_destruct(int           grid_dim,
                                           int           block_dim,
                                           cudaStream_t  stream,
                                           BufferView<T> buffer_view)
{
    // no need to destruct trivially destructible types
    if constexpr(muda::is_trivially_destructible_v<T>)
        return;

    ParallelFor(grid_dim, block_dim, size_t{0}, stream)
        .apply(static_cast<int>(buffer_view.size()),
               [buffer_view] __device__(int i) mutable
               { buffer_view.data(i)->~T(); });
}

// destruct 2D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_destruct(int             grid_dim,
                                           int             block_dim,
                                           cudaStream_t    stream,
                                           Buffer2DView<T> buffer_view)
{
    // no need to destruct trivially destructible types
    if constexpr(muda::is_trivially_destructible_v<T>)
        return;

    ParallelFor(grid_dim, block_dim, 0, stream)
        .apply(buffer_view.total_size(),
               [buffer_view] __device__(int i) mutable
               { buffer_view.data(i)->~T(); });
}

// destruct 3D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_destruct(int             grid_dim,
                                           int             block_dim,
                                           cudaStream_t    stream,
                                           Buffer3DView<T> buffer_view)
{
    // no need to destruct trivially destructible types
    if constexpr(muda::is_trivially_destructible_v<T>)
        return;

    ParallelFor(grid_dim, block_dim, 0, stream)
        .apply(buffer_view.total_size(),
               [buffer_view] __device__(int i) mutable
               { buffer_view.data(i)->~T(); });
}
}  // namespace muda::details::buffer