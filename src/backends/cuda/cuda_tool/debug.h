#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/launch.h>
#include <cuda_tool/view.h>
#include <atomic>
#include <mutex>
#include <functional>

namespace uipc::backend::cuda_tool
{
// Global debug switches + a sync callback hook (replacement for muda::Debug).
class Debug
{
  public:
    static bool debug_sync_all(bool value)
    {
        _is_debug_sync_all() = value;
        return value;
    }
    static bool is_debug_sync_all() { return _is_debug_sync_all(); }

    static void set_sync_callback(std::function<void()> callback)
    {
        std::lock_guard<std::mutex> lock(_mutex());
        _sync_callback() = std::move(callback);
    }
    static void call_sync_callback()
    {
        if(_sync_callback())
            _sync_callback()();
    }

  private:
    static std::atomic<bool>& _is_debug_sync_all()
    {
        static std::atomic<bool> v(false);
        return v;
    }
    static std::mutex& _mutex()
    {
        static std::mutex m;
        return m;
    }
    static std::function<void()>& _sync_callback()
    {
        static std::function<void()> cb = nullptr;
        return cb;
    }
};

// Synchronize the whole device and report the first CUDA error.
// Fast-fail barrier used by the simulation-dev workflow.
inline void debug_sync_all()
{
    cudaError_t e = cudaDeviceSynchronize();
    if(e != cudaSuccess)
    {
        throw std::runtime_error{
            cuda_error_string(e, "cudaDeviceSynchronize", __FILE__, __LINE__)};
    }
    Debug::call_sync_callback();
}

// device-side printf-style log tagged with a name (debugging aid)
template <typename... Args>
__device__ inline void debug_log(const char* tag, const char* fmt_str, Args&&... args)
{
    printf("[%s] ", tag);
    printf(fmt_str, std::forward<Args>(args)...);
}

// check a device buffer/view for NaN (returns true if all finite).
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
            if(!(x == x))
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
