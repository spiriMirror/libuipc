#pragma once
// muda's baseline cuda version  is 11.6
#define MUDA_BASELINE_CUDACC_VER_MAJOR 11
#define MUDA_BASELINE_CUDACC_VER_MINOR 6

#if(__CUDACC_VER_MAJOR__ >= MUDA_BASELINE_CUDACC_VER_MAJOR)                    \
    && (__CUDACC_VER_MINOR__ >= MUDA_BASELINE_CUDACC_VER_MINOR)

#define MUDA_BASELINE_CUDACC_VER_SATISFIED
#define MUDA_WITH_THRUST_UNIVERSAL
#define MUDA_WITH_GRAPH_MEMORY_ALLOC_FREE

#endif


#if(__CUDACC_VER_MAJOR__ >= 11) && (__CUDACC_VER_MINOR__ >= 2)

#define MUDA_WITH_ASYNC_MEMORY_ALLOC_FREE
namespace muda
{
constexpr bool DEFAULT_ASYNC_ALLOC_FREE = true;
}
#else
namespace muda
{
constexpr bool DEFAULT_ASYNC_ALLOC_FREE = false;
}
#endif

#if(__CUDACC_VER_MAJOR__ >= 12) && (__CUDACC_VER_MINOR__ >= 0)
#define MUDA_WITH_DEVICE_STREAM_MODEL 1
#else
#define MUDA_WITH_DEVICE_STREAM_MODEL 0
#endif