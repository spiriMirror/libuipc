#pragma once
#include <muda/launch/memory.h>
#include <muda/buffer/buffer_view.h>
#include <muda/buffer/buffer_2d_view.h>
#include <muda/buffer/buffer_3d_view.h>

namespace muda::details::buffer
{
template <typename T>
MUDA_INLINE MUDA_HOST BufferView<T> reserve_1d(cudaStream_t stream, size_t size)
{
    T* ptr = nullptr;
    Memory(stream).alloc_1d(&ptr, size * sizeof(T));
    return BufferView<T>{ptr, 0, size};
}

template <typename T>
MUDA_INLINE MUDA_HOST Buffer2DView<T> reserve_2d(cudaStream_t stream, Extent2D extent)
{
    T*     ptr         = nullptr;
    size_t pitch_bytes = 0;
    Memory(stream).alloc_2d(
        &ptr, &pitch_bytes, extent.width() * sizeof(T), extent.height());
    return Buffer2DView<T>{ptr, pitch_bytes, Offset2D::Zero(), extent};
}

template <typename T>
MUDA_INLINE MUDA_HOST Buffer3DView<T> reserve_3d(cudaStream_t stream, Extent3D extent)
{
    cudaPitchedPtr pitched_ptr;
    Memory(stream).alloc_3d(&pitched_ptr, extent.template cuda_extent<T>());
    T*     ptr              = reinterpret_cast<T*>(pitched_ptr.ptr);
    size_t pitch_bytes      = pitched_ptr.pitch;
    size_t pitch_bytes_area = pitched_ptr.pitch * extent.height();
    return Buffer3DView<T>{ptr, pitch_bytes, pitch_bytes_area, Offset3D::Zero(), extent};
}
}  // namespace muda::details::buffer