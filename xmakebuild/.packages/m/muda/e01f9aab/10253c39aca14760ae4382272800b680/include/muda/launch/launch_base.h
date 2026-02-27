#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>

#include <string>
#include <functional>
#include <memory>
#include <cooperative_groups.h>

#include <cuda_profiler_api.h>

#include <muda/type_traits/type_modifier.h>
#include <muda/tools/launch_info_cache.h>

#include <muda/check/check_cuda_errors.h>
#include <muda/muda_def.h>
#include <muda/launch/event.h>
#include <muda/launch/kernel_tag.h>

namespace muda
{
namespace details
{
    inline void stream_error_callback(cudaStream_t stream, cudaError error, void* userdata)
    {
        auto callback =
            reinterpret_cast<std::function<void(cudaStream_t, cudaError)>*>(userdata);
        (*callback)(stream, error);
        delete callback;
    }
}  // namespace details

class ComputeGraphVarBase;

template <typename T>
class ComputeGraphVar;

class LaunchCore
{
  protected:
    template <typename T>
    using S = std::shared_ptr<T>;
    MUDA_GENERIC ::cudaStream_t stream() const { return m_stream; }

    ::cudaStream_t m_stream;
    MUDA_HOST void pop_kernel_label();

  public:
    static void kernel_name(std::string_view name);
    static void file_line(std::string_view file, int line);

    MUDA_GENERIC LaunchCore(::cudaStream_t stream) MUDA_NOEXCEPT;

    void init_stream(::cudaStream_t s) { m_stream = s; }

    void push_range(const std::string& name);
    void pop_range();

    void record(cudaEvent_t e, int flag = cudaEventRecordDefault);
    void record(ComputeGraphVar<cudaEvent_t>&            e,
                const std::vector<ComputeGraphVarBase*>& vars);
    template <typename... ViewT>
    void record(ComputeGraphVar<cudaEvent_t>& e, ComputeGraphVar<ViewT>&... vars);
    void when(cudaEvent_t e, int flag = cudaEventWaitDefault);
    // let the host wait for the event
    void wait(cudaEvent_t e, int flag = cudaEventWaitDefault);
    void wait(const ComputeGraphVar<cudaEvent_t>&      e,
              const std::vector<ComputeGraphVarBase*>& vars);
    template <typename... ViewT>
    void wait(const ComputeGraphVar<cudaEvent_t>& e, ComputeGraphVar<ViewT>&... vars);
    void wait();
    void callback(const std::function<void(::cudaStream_t, ::cudaError)>& callback);

    static void wait_event(cudaEvent_t event);
    static void wait_stream(::cudaStream_t stream);
    static void wait_device();

    ~LaunchCore() MUDA_NOEXCEPT;
};

template <typename T>
class LaunchBase : public LaunchCore
{
    template <typename Others>
    friend class LaunchBase;
    using Base = LaunchCore;

  public:
    using derived_type = T;
    MUDA_GENERIC LaunchBase(::cudaStream_t stream) MUDA_NOEXCEPT;

    // create a named scope for better recognization (if you are using some profile tools)
    // usage:
    //  on(stream)
    //      .push_range("part1")
    //      .next<launch>(1,1).apply(...)
    //      .pop_range()
    //      .wait();
    T& push_range(const std::string& name);
    T& pop_range();

    // create a name for the following kernel launch
    // viewers will record this name for the sake of better recognization when debugging
    T& kernel_name(std::string_view name);
    T& file_line(std::string_view file, int line);

    // record an event on this point with current stream, you could use .when() to
    // capture this event for synchronization
    // flags:
    //  cudaEventRecordDefault : Default event creation flag.
    //  cudaEventRecordExternal : Event is captured in the graph as an external
    //  event node when performing stream capture.
    T& record(cudaEvent_t e, int flag = cudaEventRecordDefault);

    T& record(ComputeGraphVar<cudaEvent_t>&            e,
              const std::vector<ComputeGraphVarBase*>& vars);

    template <typename... ViewT>
    T& record(ComputeGraphVar<cudaEvent_t>& e, ComputeGraphVar<ViewT>&... vars);

    // let the following kernels wait until the event is triggered
    // (asynchronize with the host)
    // usage:
    //  on(stream)
    //      .when(event)
    //      .next<launch>(1,1).apply(...)
    //      .wait();
    // flags:
    //  cudaEventRecordDefault : Default event creation flag.
    //  cudaEventRecordExternal : Event is captured in the graph as an external
    //  event node when performing stream capture.
    T& when(cudaEvent_t e, int flag = cudaEventWaitDefault);
    // let the host wait for the event
    T& wait(cudaEvent_t e, int flag = cudaEventWaitDefault);
    T& wait(const ComputeGraphVar<cudaEvent_t>&      e,
            const std::vector<ComputeGraphVarBase*>& vars);
    template <typename... ViewT>
    T& wait(const ComputeGraphVar<cudaEvent_t>& e, ComputeGraphVar<ViewT>&... vars);


    // let the host wait for the current stream
    T& wait();

    // register a host callback function, which will be called when all the jobs before
    // this point are done.
    T& callback(const std::function<void(::cudaStream_t, ::cudaError)>& callback);

    template <typename Next>
    Next next(Next n);
    template <typename Next, typename... Args>
    Next next(Args&&... args);

    ~LaunchBase() MUDA_NOEXCEPT;

  protected:
    T& pop_kernel_label();

  private:
    T& derived() MUDA_NOEXCEPT { return *(T*)(this); }
};

class Empty : public LaunchBase<Empty>
{
  public:
    Empty(::cudaStream_t stream = nullptr)
        : LaunchBase(stream)
    {
    }
};

Empty on(::cudaStream_t stream);

Empty on();

void wait_device();
void wait_stream(::cudaStream_t stream);
void wait_event(cudaEvent_t event);
}  // namespace muda

#include "details/launch_base.inl"