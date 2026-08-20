#pragma once
#include <cuda_tool/stream.h>
#include <cuda_tool/launch.h>
#include <cuda_tool/view.h>
#include <uipc/common/config.h>
#include <atomic>
#include <mutex>
#include <functional>
#include <cstdlib>

namespace uipc::backend::cuda_tool
{
// Global debug switches + a sync callback hook (replacement for cuda_tool::Debug).
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

// muda compatibility alias: kernel asserts follow the project's runtime-check flag
inline constexpr bool RUNTIME_CHECK_ON = uipc::RUNTIME_CHECK;

// device-side printf-style log tagged with a name (debugging aid)
template <typename... Args>
__device__ inline void debug_log(const char* tag, const char* fmt_str, Args&&... args)
{
    printf("[%s] ", tag);
    printf(fmt_str, std::forward<Args>(args)...);
}

// ---------------------------------------------------------------------------
// trap(): device-side abort used by the kernel assert/error macros below.
// ---------------------------------------------------------------------------
__host__ __device__ inline void debug_trap() noexcept
{
#ifdef __CUDA_ARCH__
    __trap();
#else
    std::abort();
#endif
}
}  // namespace uipc::backend::cuda_tool

// ---------------------------------------------------------------------------
// Kernel diagnostics macros (replacement for the muda MUDA_KERNEL_* family).
//
// UIPC_KERNEL_ASSERT is gated on uipc::RUNTIME_CHECK (the same flag that gates
// the host-side UIPC_ASSERT), so it is live in this project's default builds.
// UIPC_KERNEL_ERROR_* always prints and traps.
// ---------------------------------------------------------------------------
#ifdef _MSC_VER
#define UIPC_FUNCTION_SIG __FUNCSIG__
#else
#define UIPC_FUNCTION_SIG __PRETTY_FUNCTION__
#endif

#ifdef __CUDA_ARCH__
#define UIPC_KERNEL_PRINT(fmt, ...)                                           \
    {                                                                         \
        ::printf("(%d|%d)-(%d|%d):" fmt "\n",                                 \
                 blockIdx.x,                                                  \
                 gridDim.x,                                                   \
                 threadIdx.x,                                                 \
                 blockDim.x,                                                  \
                 ##__VA_ARGS__);                                              \
    }
#else
#define UIPC_KERNEL_PRINT(fmt, ...)                                           \
    {                                                                         \
        ::printf("(host):" fmt "\n", ##__VA_ARGS__);                          \
    }
#endif

#define UIPC_KERNEL_ASSERT(res, fmt, ...)                                         \
    {                                                                             \
        if constexpr(::uipc::RUNTIME_CHECK)                                       \
        {                                                                         \
            if(!(res))                                                            \
            {                                                                     \
                UIPC_KERNEL_PRINT("%s(%d): %s:\n <assert> " #res " failed. " fmt, \
                                  __FILE__,                                       \
                                  __LINE__,                                       \
                                  UIPC_FUNCTION_SIG,                              \
                                  ##__VA_ARGS__);                                 \
                ::uipc::backend::cuda_tool::debug_trap();                         \
            }                                                                     \
        }                                                                         \
    }

#define UIPC_KERNEL_ERROR_WITH_LOCATION(fmt, ...)                                  \
    {                                                                              \
        UIPC_KERNEL_PRINT("%s(%d): %s:\n <error> " fmt,                            \
                          __FILE__,                                                \
                          __LINE__,                                                \
                          UIPC_FUNCTION_SIG,                                       \
                          ##__VA_ARGS__);                                          \
        ::uipc::backend::cuda_tool::debug_trap();                                  \
    }

#define UIPC_KERNEL_WARN(fmt, ...)                                                \
    {                                                                             \
        UIPC_KERNEL_PRINT("<warn>" fmt, ##__VA_ARGS__);                           \
    }

#define UIPC_KERNEL_WARN_WITH_LOCATION(fmt, ...)                                  \
    {                                                                             \
        UIPC_KERNEL_PRINT("%s(%d): %s:\n <warn>" fmt,                             \
                          __FILE__,                                               \
                          __LINE__,                                               \
                          UIPC_FUNCTION_SIG,                                      \
                          ##__VA_ARGS__);                                         \
    }
