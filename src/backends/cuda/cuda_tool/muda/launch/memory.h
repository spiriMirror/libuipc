#pragma once
#include <muda/launch/launch_base.h>
#include <muda/tools/version.h>

namespace muda
{
class Memory : public LaunchBase<Memory>
{
  public:
    MUDA_HOST Memory(cudaStream_t stream = nullptr)
        : LaunchBase(stream){};

    // Memory1D
    template <typename T>
    MUDA_HOST Memory& alloc_1d(T** ptr, size_t byte_size, bool async = DEFAULT_ASYNC_ALLOC_FREE);
    template <typename T>
    MUDA_HOST Memory& alloc(T** ptr, size_t byte_size, bool async = DEFAULT_ASYNC_ALLOC_FREE);
    MUDA_HOST Memory& free(void* ptr, bool async = DEFAULT_ASYNC_ALLOC_FREE);
    MUDA_HOST Memory& copy(void* dst, const void* src, size_t byte_size, cudaMemcpyKind kind);
    MUDA_HOST Memory& transfer(void* dst, const void* src, size_t byte_size);
    MUDA_HOST Memory& download(void* dst, const void* src, size_t byte_size);
    MUDA_HOST Memory& upload(void* dst, const void* src, size_t byte_size);
    MUDA_HOST Memory& set(void* data, size_t byte_size, char value = 0);

    // Memory2D
    template <typename T>
    MUDA_HOST Memory& alloc_2d(T**     ptr,
                               size_t* pitch,
                               size_t  width_bytes,
                               size_t  height,
                               bool    async = DEFAULT_ASYNC_ALLOC_FREE);
    template <typename T>
    MUDA_HOST Memory& alloc(T**     ptr,
                            size_t* pitch,
                            size_t  width_bytes,
                            size_t  height,
                            bool    async = DEFAULT_ASYNC_ALLOC_FREE);
    MUDA_HOST Memory& copy(void*          dst,
                           size_t         dst_pitch,
                           const void*    src,
                           size_t         src_pitch,
                           size_t         width_bytes,
                           size_t         height,
                           cudaMemcpyKind kind);
    MUDA_HOST Memory& transfer(void*       dst,
                               size_t      dst_pitch,
                               const void* src,
                               size_t      src_pitch,
                               size_t      width_bytes,
                               size_t      height);
    MUDA_HOST Memory& download(void*       dst,
                               size_t      dst_pitch,
                               const void* src,
                               size_t      src_pitch,
                               size_t      width_bytes,
                               size_t      height);
    MUDA_HOST Memory& upload(void*       dst,
                             size_t      dst_pitch,
                             const void* src,
                             size_t      src_pitch,
                             size_t      width_bytes,
                             size_t      height);
    MUDA_HOST Memory& set(void* data, size_t pitch, size_t width_bytes, size_t height, char value = 0);

    // Memory3D
    MUDA_HOST Memory& alloc_3d(cudaPitchedPtr*   pitched_ptr,
                               const cudaExtent& extent,
                               bool async = DEFAULT_ASYNC_ALLOC_FREE);
    MUDA_HOST Memory& alloc(cudaPitchedPtr*   pitched_ptr,
                            const cudaExtent& extent,
                            bool              async = DEFAULT_ASYNC_ALLOC_FREE);
    MUDA_HOST Memory& free(cudaPitchedPtr pitched_ptr, bool async = DEFAULT_ASYNC_ALLOC_FREE);
    MUDA_HOST Memory& copy(const cudaMemcpy3DParms& parms);
    MUDA_HOST Memory& transfer(cudaMemcpy3DParms parms);
    MUDA_HOST Memory& download(cudaMemcpy3DParms parms);
    MUDA_HOST Memory& upload(cudaMemcpy3DParms parms);
    MUDA_HOST Memory& set(cudaPitchedPtr pitched_ptr, cudaExtent extent, char value = 0);
};

}  // namespace muda

#include "details/memory.inl"