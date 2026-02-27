#pragma once
#include <muda/compute_graph/compute_graph.h>
#include "memory.h"
namespace muda
{
template <typename T>
MUDA_HOST Memory& Memory::alloc_1d(T** ptr, size_t byte_size, bool async)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_direct_launching(),
                "alloc must be called in direct launching mode");
#ifdef MUDA_WITH_ASYNC_MEMORY_ALLOC_FREE
    if(async)
        checkCudaErrors(cudaMallocAsync(ptr, byte_size, stream()));
    else
        checkCudaErrors(cudaMalloc(ptr, byte_size));
#else
    checkCudaErrors(cudaMalloc(ptr, byte_size));
#endif
    return *this;
}

template <typename T>
MUDA_HOST Memory& Memory::alloc(T** ptr, size_t byte_size, bool async)
{
    return alloc_1d(ptr, byte_size, async);
}

MUDA_INLINE MUDA_HOST Memory& Memory::free(void* ptr, bool async)
{
#ifdef MUDA_WITH_ASYNC_MEMORY_ALLOC_FREE
    if(async)
        checkCudaErrors(cudaFreeAsync(ptr, stream()));
    else
        checkCudaErrors(cudaFree(ptr));
#else
    checkCudaErrors(cudaFree(ptr));
#endif
    return *this;
}

MUDA_INLINE MUDA_HOST Memory& Memory::copy(void* dst, const void* src, size_t byte_size, cudaMemcpyKind kind)
{
    if constexpr(COMPUTE_GRAPH_ON)
    {
        ComputeGraphBuilder::invoke_phase_actions(
            [&] {
                checkCudaErrors(cudaMemcpyAsync(dst, src, byte_size, kind, stream()));
            },
            [&]
            {
                details::ComputeGraphAccessor().set_memcpy_node(dst, src, byte_size, kind);
            });
    }
    else
    {
        checkCudaErrors(cudaMemcpyAsync(dst, src, byte_size, kind, stream()));
    }

    return *this;
}

MUDA_INLINE MUDA_HOST Memory& Memory::transfer(void* dst, const void* src, size_t byte_size)
{
    return copy(dst, src, byte_size, cudaMemcpyDeviceToDevice);
}

MUDA_INLINE MUDA_HOST Memory& Memory::download(void* dst, const void* src, size_t byte_size)
{
    return copy(dst, src, byte_size, cudaMemcpyDeviceToHost);
}

MUDA_INLINE MUDA_HOST Memory& Memory::upload(void* dst, const void* src, size_t byte_size)
{
    return copy(dst, src, byte_size, cudaMemcpyHostToDevice);
}

MUDA_INLINE MUDA_HOST Memory& Memory::set(void* data, size_t byte_size, char byte)
{
    if constexpr(COMPUTE_GRAPH_ON)
    {
        ComputeGraphBuilder::invoke_phase_actions(
            [&] {
                checkCudaErrors(cudaMemsetAsync(data, (int)byte, byte_size, stream()));
            },
            [&]
            {
                cudaMemsetParams parms = {};
                parms.dst              = data;
                parms.value            = (int)byte;
                parms.elementSize      = 1;

                parms.pitch  = byte_size;
                parms.width  = byte_size;
                parms.height = 1;
                details::ComputeGraphAccessor().set_memset_node(parms);
            });
    }
    else
    {
        checkCudaErrors(cudaMemsetAsync(data, (int)byte, byte_size, stream()));
    }
    return *this;
}

template <typename T>
MUDA_HOST Memory& Memory::alloc_2d(T** ptr, size_t* pitch, size_t width_bytes, size_t height, bool async)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_direct_launching(),
                "alloc must be called in direct launching mode");
    checkCudaErrors(cudaMallocPitch(ptr, pitch, width_bytes, height));
    return *this;
}

template <typename T>
MUDA_HOST Memory& Memory::alloc(T** ptr, size_t* pitch, size_t width_bytes, size_t height, bool async)
{
    return alloc_2d(ptr, pitch, width_bytes, height, async);
}

MUDA_INLINE MUDA_HOST Memory& Memory::copy(void*          dst,
                                           size_t         dst_pitch,
                                           const void*    src,
                                           size_t         src_pitch,
                                           size_t         width_bytes,
                                           size_t         height,
                                           cudaMemcpyKind kind)
{
    if constexpr(COMPUTE_GRAPH_ON)
    {
        ComputeGraphBuilder::invoke_phase_actions(
            [&]
            {
                checkCudaErrors(cudaMemcpy2DAsync(
                    dst, dst_pitch, src, src_pitch, width_bytes, height, kind, stream()));
            },
            [&]
            {
                cudaMemcpy3DParms parms = {};
                parms.srcPtr =
                    make_cudaPitchedPtr((void*)src, src_pitch, width_bytes, height);
                parms.dstPtr = make_cudaPitchedPtr(dst, dst_pitch, width_bytes, height);
                parms.extent = make_cudaExtent(width_bytes, height, 1);
                parms.kind   = kind;
                details::ComputeGraphAccessor().set_memcpy_node(parms);
            });
    }
    else
    {
        checkCudaErrors(cudaMemcpy2DAsync(
            dst, dst_pitch, src, src_pitch, width_bytes, height, kind, stream()));
    }

    return *this;
}

MUDA_INLINE MUDA_HOST Memory& Memory::transfer(void*       dst,
                                               size_t      dst_pitch,
                                               const void* src,
                                               size_t      src_pitch,
                                               size_t      width_bytes,
                                               size_t      height)
{
    return copy(dst, dst_pitch, src, src_pitch, width_bytes, height, cudaMemcpyDeviceToDevice);
}

MUDA_INLINE MUDA_HOST Memory& Memory::download(void*       dst,
                                               size_t      dst_pitch,
                                               const void* src,
                                               size_t      src_pitch,
                                               size_t      width_bytes,
                                               size_t      height)
{
    return copy(dst, dst_pitch, src, src_pitch, width_bytes, height, cudaMemcpyDeviceToHost);
}

MUDA_INLINE MUDA_HOST Memory& Memory::upload(void*       dst,
                                             size_t      dst_pitch,
                                             const void* src,
                                             size_t      src_pitch,
                                             size_t      width_bytes,
                                             size_t      height)
{
    return copy(dst, dst_pitch, src, src_pitch, width_bytes, height, cudaMemcpyHostToDevice);
}

MUDA_INLINE MUDA_HOST Memory& Memory::set(
    void* data, size_t pitch, size_t width_bytes, size_t height, char value)
{
    if constexpr(COMPUTE_GRAPH_ON)
    {
        ComputeGraphBuilder::invoke_phase_actions(
            [&]
            {
                checkCudaErrors(cudaMemset2DAsync(
                    data, pitch, (int)value, width_bytes, height, stream()));
            },
            [&]
            {
                cudaMemsetParams parms = {};
                parms.dst              = data;
                parms.value            = (int)value;
                parms.elementSize      = sizeof(char);

                parms.pitch  = pitch;
                parms.width  = width_bytes;
                parms.height = height;
                details::ComputeGraphAccessor().set_memset_node(parms);
            });
    }
    else
    {
        checkCudaErrors(
            cudaMemset2DAsync(data, pitch, (int)value, width_bytes, height, stream()));
    }
    return *this;
}


MUDA_INLINE MUDA_HOST Memory& Memory::alloc_3d(cudaPitchedPtr*   pitched_ptr,
                                               const cudaExtent& extent,
                                               bool              async)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_direct_launching(),
                "alloc must be called in direct launching mode");
    checkCudaErrors(cudaMalloc3D(pitched_ptr, extent));
    return *this;
}

MUDA_INLINE MUDA_HOST Memory& Memory::alloc(cudaPitchedPtr*   pitched_ptr,
                                            const cudaExtent& extent,
                                            bool              async)
{
    return alloc_3d(pitched_ptr, extent, async);
}

MUDA_INLINE MUDA_HOST Memory& Memory::free(cudaPitchedPtr pitched_ptr, bool async)
{
    return free(pitched_ptr.ptr, async);
}

MUDA_INLINE MUDA_HOST Memory& Memory::copy(const cudaMemcpy3DParms& parms)
{
    if constexpr(COMPUTE_GRAPH_ON)
    {
        ComputeGraphBuilder::invoke_phase_actions(
            [&] { checkCudaErrors(cudaMemcpy3DAsync(&parms, stream())); },
            [&] { details::ComputeGraphAccessor().set_memcpy_node(parms); });
    }
    else
    {
        checkCudaErrors(cudaMemcpy3DAsync(&parms, stream()));
    }
    return *this;
}

MUDA_INLINE MUDA_HOST Memory& Memory::transfer(cudaMemcpy3DParms parms)
{
    parms.kind = cudaMemcpyDeviceToDevice;
    return copy(parms);
}

MUDA_INLINE MUDA_HOST Memory& Memory::download(cudaMemcpy3DParms parms)
{
    parms.kind = cudaMemcpyDeviceToHost;
    return copy(parms);
}
MUDA_INLINE MUDA_HOST Memory& Memory::upload(cudaMemcpy3DParms parms)
{
    parms.kind = cudaMemcpyHostToDevice;
    return copy(parms);
}

MUDA_INLINE MUDA_HOST Memory& Memory::set(cudaPitchedPtr pitched_ptr, cudaExtent extent, char value)
{
    if constexpr(COMPUTE_GRAPH_ON)
    {
        ComputeGraphBuilder::invoke_phase_actions(
            [&]
            {
                checkCudaErrors(cudaMemset3DAsync(pitched_ptr, (int)value, extent, stream()));
            },
            [&]
            {
                // seems unable to set a 3D memory in cudaGraph (no depth parameter)
                // so we capture cudaMemset3DAsync instead
                ComputeGraphBuilder::capture(
                    enum_name(ComputeGraphNodeType::MemsetNode),
                    [&](cudaStream_t stream) {
                        cudaMemset3DAsync(pitched_ptr, (int)value, extent, stream);
                    });
            });
    }
    else
    {
        checkCudaErrors(cudaMemset3DAsync(pitched_ptr, (int)value, extent, stream()));
    }
    return *this;
}
}  // namespace muda