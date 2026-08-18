#include <muda/launch/stream_define.h>
namespace muda
{
MUDA_INLINE MUDA_GENERIC GraphViewer::GraphViewer(cudaGraphExec_t graph,
                                                  Flags<GraphInstantiateFlagBit> flags)
    : m_graph(graph)
    , m_flags(flags)
{
}

MUDA_INLINE MUDA_GENERIC void GraphViewer::launch(cudaStream_t stream) const
{
#ifdef __CUDA_ARCH__
    MUDA_KERNEL_ASSERT(m_flags.has(GraphInstantiateFlagBit::DeviceLaunch),
                       "To launch on device, the graph should be instantiated with `DeviceLaunch`");
    MUDA_KERNEL_ASSERT(stream == details::stream::graph_tail_launch()
                           || stream == details::stream::graph_fire_and_forget(),
                       "Launch Graph on device with invalid stream! "
                       "Only Stream::GraphTailLaunch{} and Stream::GraphFireAndForget{} are allowed");
#if !MUDA_WITH_DEVICE_STREAM_MODEL
    MUDA_ERROR_WITH_LOCATION(
        "GraphViewer[%s:%s]: graph launch on device is not supported in "
        "cuda %d-%d.",
        kernel_name(),
        name(),
        __CUDACC_VER_MAJOR__,
        __CUDACC_VER_MINOR__);
#endif
#endif
    auto graph_viewer_error_code = cudaGraphLaunch(m_graph, stream);
    if(graph_viewer_error_code != cudaSuccess)
    {
        auto error_string = cudaGetErrorString(graph_viewer_error_code);
        MUDA_KERNEL_ERROR_WITH_LOCATION("GraphViewer[%s:%s]: launch error: %s(%d), GraphExec=%p",
                                        kernel_name(),
                                        name(),
                                        error_string,
                                        (int)graph_viewer_error_code,
                                        m_graph);
    }
}

MUDA_INLINE MUDA_DEVICE void GraphViewer::tail_launch() const
{
#ifdef __CUDA_ARCH__
    this->launch(details::stream::graph_tail_launch());
#endif
}

MUDA_INLINE MUDA_DEVICE void GraphViewer::fire_and_forget() const
{
#ifdef __CUDA_ARCH__
    this->launch(details::stream::graph_fire_and_forget());
#endif
}
}  // namespace muda
