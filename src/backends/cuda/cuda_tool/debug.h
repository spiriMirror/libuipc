#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/launch.h>
#include <cuda_tool/view.h>

namespace uipc::backend::cuda_tool
{
// Synchronize and report the first CUDA error across all active streams.
// Debugging aid used by the simulation-dev workflow (fast-fail barrier).
inline void debug_sync_all()
{
    cudaError_t e = cudaDeviceSynchronize();
    if(e != cudaSuccess)
    {
        throw std::runtime_error{
            cuda_error_string(e, "cudaDeviceSynchronize", __FILE__, __LINE__)};
    }
}

// device-side printf-style log tagged with a name (debugging aid)
template <typename... Args>
__device__ inline void debug_log(const char* tag, const char* fmt_str, Args&&... args)
{
    printf("[%s] ", tag);
    printf(fmt_str, std::forward<Args>(args)...);
}

// check a device buffer/view for NaN/Inf by reducing a flag on host.
// returns true if any non-finite value is found.
template <typename View>
inline bool check_finite(View v, cudaStream_t s = default_stream())
{
    int* d_flag = nullptr;
    CUDA_TOOL_CHECK(cudaMalloc(&d_flag, sizeof(int)));
    CUDA_TOOL_CHECK(cudaMemsetAsync(d_flag, 0, sizeof(int), s));
    parallel_for(
        (int)v.size(),
        [v, d_flag] __device__(int i) mutable
        {
            auto x = v[i];
            if(!(x == x))  // NaN
                atomicExch(d_flag, 1);
        },
        s);
    int h_flag = 0;
    CUDA_TOOL_CHECK(
        cudaMemcpyAsync(&h_flag, d_flag, sizeof(int), cudaMemcpyDeviceToHost, s));
    CUDA_TOOL_CHECK(cudaStreamSynchronize(s));
    cudaFree(d_flag);
    return h_flag == 0;
}
}  // namespace uipc::backend::cuda_tool
