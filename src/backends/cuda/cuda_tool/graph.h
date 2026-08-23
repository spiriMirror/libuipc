#pragma once
// ---------------------------------------------------------------------------
// Minimal CUDA-graph block capture/replay helper (host-side only).
//
// Records a fixed sequence of stream operations once and replays it as a
// single cudaGraphLaunch, removing per-launch gaps between small kernels.
//
// Compatibility (verified against the NVIDIA CUDA Runtime API docs,
// Graph Management section, per-version archives):
//   - stream capture / cudaGraphLaunch / cudaGraph(Destroy|ExecDestroy):
//     available since CUDA 10.0, unchanged through 13.x.
//   - instantiation:
//       CUDA >= 12.0:  cudaGraphInstantiate(pGraphExec, graph, flags)
//                      (errorNode/logBuffer args REMOVED vs 11.x);
//                      cudaGraphInstantiateWithFlags(exec, graph, flags)
//                      exists since 11.4 and is still valid in 12.x/13.x.
//       11.4 - 11.8:   cudaGraphInstantiateWithFlags (same signature).
//       10.x - 11.3:   only the legacy 5-arg cudaGraphInstantiate(exec,
//                      graph, pErrorNode, pLogBuffer, bufferSize) exists.
//   The dispatch below therefore covers 10.x/11.x/12.x/13.x at compile time.
//   Tested on 12.8 and 13.2.
//
// Contract / traps (see agent_docs/08):
//  - everything the graph consumes must be launched on the stream handed to
//    the capture body; a legacy-default-stream launch inside the body
//    invalidates the capture (reported as Result::CaptureInvalidated).
//  - kernel arguments are baked at capture time, including by-value counts
//    inside views — the caller must key validity on anything that can change.
//  - no host-side reads of the captured region's outputs during capture.
// ---------------------------------------------------------------------------
#include <cuda_runtime.h>

namespace uipc::backend::cuda_tool
{
class GraphCapture
{
  public:
    enum class Result
    {
        Ok,
        BeginCaptureFailed,
        CaptureInvalidated,  // a callee escaped the capture stream
        InstantiateFailed,
    };

    GraphCapture() = default;
    ~GraphCapture() { reset(); }

    GraphCapture(const GraphCapture&)            = delete;
    GraphCapture& operator=(const GraphCapture&) = delete;

    // Record body(capture_stream) into a graph; nothing is executed during
    // capture. On any failure the instance is disabled permanently (future
    // capture() calls return the stored failure immediately) and ready()
    // stays false.
    template <typename F>
    Result capture(F&& body)
    {
        if(m_disabled)
            return m_failure;

        reset_graph();

        if(!m_capture_stream)
            cudaStreamCreateWithFlags(&m_capture_stream, cudaStreamNonBlocking);

        cudaError_t err =
            cudaStreamBeginCapture(m_capture_stream, cudaStreamCaptureModeThreadLocal);
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            return fail(Result::BeginCaptureFailed);
        }

        body(m_capture_stream);

        err = cudaStreamEndCapture(m_capture_stream, &m_graph);
        if(err != cudaSuccess || m_graph == nullptr)
        {
            cudaGetLastError();
            m_graph = nullptr;
            return fail(Result::CaptureInvalidated);
        }

#if CUDART_VERSION >= 11040
        err = cudaGraphInstantiateWithFlags(&m_exec, m_graph, 0);
#else
        err = cudaGraphInstantiate(&m_exec, m_graph, nullptr, nullptr, 0);
#endif
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            reset_graph();
            return fail(Result::InstantiateFailed);
        }
        return Result::Ok;
    }

    // Replay the captured graph on an internal blocking stream and wait for
    // completion. (A blocking stream does NOT wait for legacy-default-stream
    // work — make sure any async H2D feeding the graph has completed, e.g. by
    // using a synchronous cudaMemcpy.)
    void launch_sync()
    {
        if(!m_launch_stream)
            cudaStreamCreateWithFlags(&m_launch_stream, cudaStreamDefault);
        cudaGraphLaunch(m_exec, m_launch_stream);
        cudaStreamSynchronize(m_launch_stream);
    }

    bool ready() const { return m_exec != nullptr; }
    bool disabled() const { return m_disabled; }

    // Drop the captured graph (streams are kept for reuse).
    void reset_graph()
    {
        if(m_exec)
        {
            cudaGraphExecDestroy(m_exec);
            m_exec = nullptr;
        }
        if(m_graph)
        {
            cudaGraphDestroy(m_graph);
            m_graph = nullptr;
        }
    }

    void reset()
    {
        reset_graph();
        if(m_capture_stream)
        {
            cudaStreamDestroy(m_capture_stream);
            m_capture_stream = nullptr;
        }
        if(m_launch_stream)
        {
            cudaStreamDestroy(m_launch_stream);
            m_launch_stream = nullptr;
        }
    }

  private:
    Result fail(Result r)
    {
        m_failure  = r;
        m_disabled = true;
        return r;
    }

    cudaStream_t    m_capture_stream = nullptr;
    cudaStream_t    m_launch_stream  = nullptr;
    cudaGraph_t     m_graph          = nullptr;
    cudaGraphExec_t m_exec           = nullptr;
    bool            m_disabled       = false;
    Result          m_failure        = Result::Ok;
};
}  // namespace uipc::backend::cuda_tool
