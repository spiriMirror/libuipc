#include <muda/exception.h>
#if MUDA_WITH_NVTX3
#include <nvtx3/nvToolsExt.h>
#include <nvtx3/nvToolsExtCuda.h>
#endif
#include <muda/compute_graph/compute_graph_accessor.h>
#include <muda/compute_graph/compute_graph_var.h>
#include <muda/graph/graph.h>
#include <iostream>

namespace muda
{
MUDA_INLINE MUDA_GENERIC LaunchCore::LaunchCore(cudaStream_t stream) MUDA_NOEXCEPT
    : m_stream(stream)
{
//Logger::instance();
#ifdef __CUDA_ARCH__
#else
    if(!ComputeGraphBuilder::is_phase_none())
    {
        if(ComputeGraphBuilder::is_phase_serial_launching())
        {
            MUDA_ASSERT(stream == nullptr
                            || stream == details::ComputeGraphAccessor().current_stream(),
                        "LaunchBase: stream must be nullptr or equals to current stream");
            init_stream(details::ComputeGraphAccessor().current_stream());
        }
        else if(ComputeGraphBuilder::is_caturing())
        {
            init_stream(details::ComputeGraphAccessor().capture_stream());
        }
    }
#endif
}


MUDA_INLINE void LaunchCore::push_range(const std::string& name)
{
#if MUDA_WITH_NVTX3
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "`push_range()` is meaningless in ComputeGraph");

    nvtxEventAttributes_t eventAttrib = {0};
    eventAttrib.version               = NVTX_VERSION;
    eventAttrib.size                  = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    eventAttrib.colorType             = NVTX_COLOR_ARGB;
    eventAttrib.color                 = 255;
    eventAttrib.messageType           = NVTX_MESSAGE_TYPE_ASCII;
    eventAttrib.message.ascii         = name.c_str();
    nvtxRangePushEx(&eventAttrib);
#endif
}

MUDA_INLINE void LaunchCore::pop_range()
{
#if MUDA_WITH_NVTX3
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "`pop_range()` is meaningless in ComputeGraph");
    nvtxRangePop();
#endif
}


MUDA_INLINE void LaunchCore::record(cudaEvent_t e, int flag)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "You need provide at least one ComputeGraphVar for dependency generation");
    checkCudaErrors(cudaEventRecordWithFlags(e, stream(), flag));
}

MUDA_INLINE void LaunchCore::record(ComputeGraphVar<cudaEvent_t>& e,
                                    const std::vector<ComputeGraphVarBase*>& vars)
{
    auto event = e.eval();
    for(auto var : vars)
        var->base_building_eval();  // eval all vars (for safety, we eval them as RWViewer)
    ComputeGraphBuilder::invoke_phase_actions(
        [&]
        {
            checkCudaErrors(cudaEventRecordWithFlags(event, m_stream, cudaEventRecordDefault));
        },
        [&] { details::ComputeGraphAccessor().set_event_record_node(event); },
        [&] { details::ComputeGraphAccessor().set_event_record_node(nullptr); });
}

MUDA_INLINE void LaunchCore::when(cudaEvent_t e, int flag)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "`when()` makes code reader confused in ComputeGraph, please use `wait()` instead")
    checkCudaErrors(cudaStreamWaitEvent(stream(), e, flag));
}

MUDA_INLINE void LaunchCore::wait(cudaEvent_t e, int flag)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "You need provide at least one ComputeGraphVar for dependency generation");

    checkCudaErrors(cudaStreamWaitEvent(m_stream, e, flag));
}

MUDA_INLINE void LaunchCore::wait(const ComputeGraphVar<cudaEvent_t>&      e,
                                  const std::vector<ComputeGraphVarBase*>& vars)
{
    auto event = e.ceval();
    for(auto var : vars)
        var->base_building_eval();  // eval all vars (for safety, we eval them as RWViewer)
    ComputeGraphBuilder::invoke_phase_actions(
        [&]
        {
            checkCudaErrors(cudaStreamWaitEvent(m_stream, event, cudaEventWaitDefault));
        },
        [&] { details::ComputeGraphAccessor().set_event_wait_node(event); },
        [&] { details::ComputeGraphAccessor().set_event_wait_node(nullptr); });
}

MUDA_INLINE void LaunchCore::wait()
{
    wait_stream(m_stream);
}

MUDA_INLINE void LaunchCore::callback(const std::function<void(cudaStream_t, cudaError)>& callback)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "`callback()` in ComputeGraph is unsupported now");
    auto userdata = new std::function<void(cudaStream_t, cudaError)>(callback);
    checkCudaErrors(
        cudaStreamAddCallback(stream(), details::stream_error_callback, userdata, 0));
}

template <typename... ViewT>
MUDA_INLINE void LaunchCore::record(ComputeGraphVar<cudaEvent_t>& e,
                                    ComputeGraphVar<ViewT>&... vars)
{
    record(e, {static_cast<ComputeGraphVarBase*>(&vars)...});
}


template <typename... ViewT>
MUDA_INLINE void LaunchCore::wait(const ComputeGraphVar<cudaEvent_t>& e,
                                  ComputeGraphVar<ViewT>&... vars)
{
    return wait(e, {static_cast<ComputeGraphVarBase*>(&vars)...});
}

MUDA_INLINE void LaunchCore::kernel_name(std::string_view name)
{
    if constexpr(muda::RUNTIME_CHECK_ON)
        details::LaunchInfoCache::current_kernel_name(name);
}

MUDA_INLINE void muda::LaunchCore::file_line(std::string_view file, int line)
{
    if constexpr(muda::RUNTIME_CHECK_ON)
    {
        details::LaunchInfoCache::current_kernel_file(file);
        details::LaunchInfoCache::current_kernel_line(line);
    }
}

MUDA_INLINE MUDA_HOST void LaunchCore::pop_kernel_label()
{
    if constexpr(muda::RUNTIME_CHECK_ON)
    {
        details::LaunchInfoCache::current_kernel_name("");
        details::LaunchInfoCache::current_kernel_file("");
        details::LaunchInfoCache::current_kernel_line(0ull);
    }
}


MUDA_INLINE LaunchCore::~LaunchCore() MUDA_NOEXCEPT
{
    if constexpr(muda::RUNTIME_CHECK_ON)
    {
        if(ComputeGraphBuilder::is_direct_launching() && Debug::is_debug_sync_all())
            wait();
    }
}

MUDA_INLINE void LaunchCore::wait_event(cudaEvent_t event)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "`wait_event()` is meaningless in ComputeGraph");
    checkCudaErrors(cudaEventSynchronize(event));

    if constexpr(muda::RUNTIME_CHECK_ON)
    {
        Debug::call_sync_callback();
    }
}

MUDA_INLINE void LaunchCore::wait_stream(cudaStream_t stream)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "`wait_stream()` a stream is meaningless in ComputeGraph");
    checkCudaErrors(cudaStreamSynchronize(stream));

    if constexpr(muda::RUNTIME_CHECK_ON)
    {
        Debug::call_sync_callback();
    }
}

MUDA_INLINE void LaunchCore::wait_device()
{
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "`wait_device()` a stream is meaningless in ComputeGraph");
    checkCudaErrors(cudaDeviceSynchronize());

    if constexpr(muda::RUNTIME_CHECK_ON)
    {
        Debug::call_sync_callback();
    }
}

template <typename T>
MUDA_GENERIC LaunchBase<T>::LaunchBase(cudaStream_t stream) MUDA_NOEXCEPT
    : LaunchCore(stream)
{
}

template <typename T>
T& LaunchBase<T>::push_range(const std::string& name)
{
    LaunchCore::push_range(name);
    return derived();
}

template <typename T>
T& LaunchBase<T>::pop_range()
{
    LaunchCore::pop_range();
    return derived();
}

template <typename T>
T& LaunchBase<T>::record(cudaEvent_t e, int flag)
{
    LaunchCore::record(e, flag);
    return derived();
}

template <typename T>
T& LaunchBase<T>::record(ComputeGraphVar<cudaEvent_t>&            e,
                         const std::vector<ComputeGraphVarBase*>& vars)
{
    LaunchCore::record(e, vars);
    return derived();
}

template <typename T>
T& LaunchBase<T>::when(cudaEvent_t e, int flag)
{
    LaunchCore::when(e, flag);
    return derived();
}

template <typename T>
T& LaunchBase<T>::wait(cudaEvent_t e, int flag)
{
    LaunchCore::wait(e, flag);
    return derived();
}

template <typename T>
T& LaunchBase<T>::wait(const ComputeGraphVar<cudaEvent_t>&      e,
                       const std::vector<ComputeGraphVarBase*>& vars)
{
    LaunchCore::wait(e, vars);
    return derived();
}

template <typename T>
T& LaunchBase<T>::wait()
{
    LaunchCore::wait();
    return derived();
}

template <typename T>
T& LaunchBase<T>::callback(const std::function<void(cudaStream_t, cudaError)>& callback)
{
    LaunchCore::callback(callback);
    return derived();
}
template <typename T>
template <typename... ViewT>
T& LaunchBase<T>::record(ComputeGraphVar<cudaEvent_t>& e, ComputeGraphVar<ViewT>&... vars)
{
    return record(e, {static_cast<ComputeGraphVarBase*>(&vars)...});
}

template <typename T>
template <typename... ViewT>
T& LaunchBase<T>::wait(const ComputeGraphVar<cudaEvent_t>& e, ComputeGraphVar<ViewT>&... vars)
{
    return wait(e, {static_cast<ComputeGraphVarBase*>(&vars)...});
}

template <typename T>
template <typename Next>
Next LaunchBase<T>::next(Next n)
{
    static_assert(std::is_base_of_v<LaunchBase<Next>, Next>,
                  "Next should be derived from LaunchBase<Next>");
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(), "`next()` is not allowed in ComputeGraph");
    n.init_stream(stream());
    return n;
}

template <typename T>
template <typename Next, typename... Args>
Next LaunchBase<T>::next(Args&&... args)
{
    static_assert(std::is_base_of_v<LaunchBase<Next>, Next>,
                  "Next should be derived from LaunchBase<Next>");
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(), "`next()` is not allowed in ComputeGraph");
    Next n(std::forward<Args>(args)...);
    n.init_stream(stream());
    return n;
}

template <typename T>
T& LaunchBase<T>::kernel_name(std::string_view name)
{
    LaunchCore::kernel_name(name);
    return derived();
}

template <typename T>
T& muda::LaunchBase<T>::file_line(std::string_view file, int line)
{
    LaunchCore::file_line(file, line);
    return derived();
}

template <typename T>
T& LaunchBase<T>::pop_kernel_label()
{
    LaunchCore::pop_kernel_label();
    return derived();
}

template <typename T>
LaunchBase<T>::~LaunchBase() MUDA_NOEXCEPT
{
}

MUDA_INLINE Empty on(cudaStream_t stream)
{
    MUDA_ASSERT(ComputeGraphBuilder::is_phase_none(),
                "`on(stream)` is meaningless in ComputeGraph, using `on()` is enough");
    return Empty(stream);
}

MUDA_INLINE Empty on()
{
    return Empty(nullptr);
}

MUDA_INLINE void wait_device()
{
    Empty::wait_device();
}

MUDA_INLINE void wait_stream(cudaStream_t stream)
{
    Empty::wait_stream(stream);
}

MUDA_INLINE void wait_event(cudaEvent_t event)
{
    Empty::wait_event(event);
}
}  // namespace muda