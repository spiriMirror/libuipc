#pragma once
#include <cuda.h>
#include <cinttypes>
#include <muda/tools/version.h>
#include <cuda_device_runtime_api.h>
namespace muda::details::stream
{
#if MUDA_GENERIC_STREAM_MODEL
MUDA_INLINE MUDA_GENERIC cudaStream_t tail_launch()
{
    return cudaStreamTailLaunch;
}
MUDA_INLINE MUDA_GENERIC cudaStream_t fire_and_forget()
{
    return cudaStreamFireAndForget;
}
MUDA_INLINE MUDA_GENERIC cudaStream_t graph_tail_launch()
{
    return cudaStreamGraphTailLaunch;
}
MUDA_INLINE MUDA_GENERIC cudaStream_t graph_fire_and_forget()
{
    return cudaStreamGraphFireAndForget;
}
#else
MUDA_INLINE MUDA_GENERIC cudaStream_t graph_tail_launch()
{
    return reinterpret_cast<cudaStream_t>(0x0100000000000000);
}
MUDA_INLINE MUDA_GENERIC cudaStream_t graph_fire_and_forget()
{
    return reinterpret_cast<cudaStream_t>(0x0200000000000000);
}
MUDA_INLINE MUDA_GENERIC cudaStream_t tail_launch()
{
    return reinterpret_cast<cudaStream_t>(0x3);
}
MUDA_INLINE MUDA_GENERIC cudaStream_t fire_and_forget()
{
    return reinterpret_cast<cudaStream_t>(0x4);
}
#endif
}  // namespace muda::details::stream
