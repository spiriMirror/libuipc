#pragma once
// CRTP launch base providing a fluent, chainable stream-bound launcher.
// Replacement for muda::LaunchBase (the algorithm backends subclass it).
#include <cuda_tool/stream.h>
#include <functional>
#include <string>
#include <string_view>

namespace uipc::backend::cuda_tool
{
// Non-template core holding the stream and stream-level operations.
class LaunchCore
{
  public:
    explicit LaunchCore(cudaStream_t stream = default_stream()) noexcept
        : m_stream(stream)
    {
    }
    virtual ~LaunchCore() = default;

    cudaStream_t stream() const { return m_stream; }

    void wait() { CUDA_TOOL_CHECK(cudaStreamSynchronize(m_stream)); }
    void callback(const std::function<void(cudaStream_t, cudaError_t)>& cb)
    {
        CUDA_TOOL_CHECK(cudaStreamAddCallback(
            m_stream,
            [](cudaStream_t s, cudaError_t e, void* user)
            { (*static_cast<std::function<void(cudaStream_t, cudaError_t)>*>(user))(s, e); },
            const_cast<std::function<void(cudaStream_t, cudaError_t)>*>(&cb),
            0));
    }

    static void wait_stream(cudaStream_t s) { CUDA_TOOL_CHECK(cudaStreamSynchronize(s)); }
    static void wait_device() { CUDA_TOOL_CHECK(cudaDeviceSynchronize()); }
    static void wait_event(cudaEvent_t e) { CUDA_TOOL_CHECK(cudaEventSynchronize(e)); }

  protected:
    void init_stream(cudaStream_t s) { m_stream = s; }
    // kernel labeling hooks are no-ops (debug metadata dropped vs muda)
    void pop_kernel_label() {}

    cudaStream_t m_stream;
};

template <typename T>
class LaunchBase : public LaunchCore
{
  public:
    using derived_type = T;
    explicit LaunchBase(cudaStream_t stream = default_stream()) noexcept
        : LaunchCore(stream)
    {
    }

    T& kernel_name(std::string_view) { return derived(); }
    T& file_line(std::string_view, int) { return derived(); }
    T& wait()
    {
        LaunchCore::wait();
        return derived();
    }
    T& callback(const std::function<void(cudaStream_t, cudaError_t)>& cb)
    {
        LaunchCore::callback(cb);
        return derived();
    }

    template <typename Next>
    Next next(Next n)
    {
        n.init_stream(this->stream());
        return n;
    }
    template <typename Next, typename... Args>
    Next next(Args&&... args)
    {
        Next n(std::forward<Args>(args)...);
        n.init_stream(this->stream());
        return n;
    }

  protected:
    T& derived() noexcept { return *static_cast<T*>(this); }
};

// free helpers matching muda's on()/wait_device()/wait_stream()
inline LaunchCore on(cudaStream_t s) { return LaunchCore{s}; }
inline LaunchCore on() { return LaunchCore{default_stream()}; }
inline void        wait_device() { LaunchCore::wait_device(); }
inline void        wait_stream(cudaStream_t s) { LaunchCore::wait_stream(s); }
}  // namespace uipc::backend::cuda_tool
