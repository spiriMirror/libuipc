#include <muda/type_traits/type_label.h>
#include <muda/launch/memory.h>
#include <muda/launch/parallel_for.h>
#include <muda/buffer/buffer_view.h>
#include <muda/buffer/buffer_2d_view.h>
#include <muda/buffer/buffer_3d_view.h>
#include <muda/buffer/buffer_info_accessor.h>

namespace muda::details::buffer
{
// copy construct 0D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_copy_construct(cudaStream_t stream,
                                                 VarView<T>   dst,
                                                 CVarView<T>  src)
{
    ParallelFor(1, 1, 0, stream)
        .apply(1,
               [dst, src] __device__(int i) mutable
               { new(dst.data()) T(*src.data()); });
}

template <typename T>
MUDA_INLINE MUDA_HOST void kernel_copy_construct_non_trivial(int grid_dim,
                                                             int block_dim,
                                                             cudaStream_t stream,
                                                             BufferView<T>& dst,
                                                             CBufferView<T>& src)
{
    ParallelFor(grid_dim, block_dim, 0, stream)
        .apply(dst.size(),
               [dst, src] __device__(int i) mutable
               { new(dst.data(i)) T(*src.data(i)); });
}

// copy construct 1D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_copy_construct(int            grid_dim,
                                                 int            block_dim,
                                                 cudaStream_t   stream,
                                                 BufferView<T>  dst,
                                                 CBufferView<T> src)
{
    if constexpr(muda::is_trivially_copy_constructible_v<T>)
    {
        // trivially copy constructible, use cudaMemcpy
        Memory(stream).transfer(dst.data(), src.data(), dst.size() * sizeof(T));
    }
    else
    {
        kernel_copy_construct_non_trivial(grid_dim, block_dim, stream, dst, src);
    }
}

template <typename T>
MUDA_INLINE MUDA_HOST void kernel_copy_construct_non_trivial(int grid_dim,
                                                             int block_dim,
                                                             cudaStream_t stream,
                                                             Buffer2DView<T>& dst,
                                                             CBuffer2DView<T>& src)
{
    ParallelFor(grid_dim, block_dim, 0, stream)
        .apply(dst.total_size(),
               [dst, src] __device__(int i) mutable
               { new(dst.data(i)) T(*src.data(i)); });
}

// copy construct 2D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_copy_construct(int              grid_dim,
                                                 int              block_dim,
                                                 cudaStream_t     stream,
                                                 Buffer2DView<T>  dst,
                                                 CBuffer2DView<T> src)
{
    if constexpr(muda::is_trivially_copy_constructible_v<T>)
    {
        // trivially copy constructible, use cudaMemcpy
        cudaMemcpy3DParms parms = {0};
        parms.srcPtr =
            details::buffer::BufferInfoAccessor::template cuda_pitched_ptr(src);
        parms.srcPos = src.offset().template cuda_pos<T>();
        parms.dstPtr =
            details::buffer::BufferInfoAccessor::template cuda_pitched_ptr(dst);
        parms.extent = dst.extent().template cuda_extent<T>();
        parms.dstPos = dst.offset().template cuda_pos<T>();

        Memory(stream).transfer(parms);
    }
    else
    {
        // non-trivially copy constructible, use placement new
        kernel_copy_construct_non_trivial(grid_dim, block_dim, stream, dst, src);
    }
}

template <typename T>
MUDA_INLINE MUDA_HOST void kernel_copy_construct_non_trivial(int grid_dim,
                                                             int block_dim,
                                                             cudaStream_t stream,
                                                             Buffer3DView<T>& dst,
                                                             CBuffer3DView<T>& src)
{
    ParallelFor(grid_dim, block_dim, 0, stream)
        .apply(dst.total_size(),
               [dst, src] __device__(int i) mutable
               { new(dst.data(i)) T(*src.data(i)); });
}

// copy construct 3D
template <typename T>
MUDA_INLINE MUDA_HOST void kernel_copy_construct(int              grid_dim,
                                                 int              block_dim,
                                                 cudaStream_t     stream,
                                                 Buffer3DView<T>  dst,
                                                 CBuffer3DView<T> src)
{
    if constexpr(muda::is_trivially_copy_constructible_v<T>)
    {
        // trivially copy constructible, use cudaMemcpy
        cudaMemcpy3DParms parms = {0};
        parms.srcPtr =
            details::buffer::BufferInfoAccessor::template cuda_pitched_ptr(src);
        parms.srcPos = src.offset().template cuda_pos<T>();
        parms.dstPtr =
            details::buffer::BufferInfoAccessor::template cuda_pitched_ptr(dst);
        parms.extent = dst.extent().template cuda_extent<T>();
        parms.dstPos = dst.offset().template cuda_pos<T>();

        Memory(stream).transfer(parms);
    }
    else
    {
        // non-trivially copy constructible, use placement new
        kernel_copy_construct_non_trivial(grid_dim, block_dim, stream, dst, src);
    }
}
}  // namespace muda::details::buffer