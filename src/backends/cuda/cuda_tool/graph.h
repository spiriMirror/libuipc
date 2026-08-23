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
//   Also note cudaGraphAddNode gained the edge-data parameter in CUDA 13.0
//   (5-arg form before) — dispatched at the call site via CUDART_VERSION.
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
#include <string>

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
            // blocking on purpose: any callee launching into the legacy
            // default stream during capture then INVALIDATES the capture
            // (our automatic fallback signal). With a non-blocking capture
            // stream, such kernels would silently EXECUTE during capture and
            // be missing from the graph — the replayed graph would run the
            // solver with the preconditioner frozen at its capture-time
            // output (this produced false PCG convergence on MAS scenes).
            cudaStreamCreateWithFlags(&m_capture_stream, cudaStreamDefault);

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

// ---------------------------------------------------------------------------
// GraphWhile — a WHILE-loop graph (CUDA >= 12.4 toolkit + driver).
//
// The whole iterative loop lives on device: `capture(setup, body)` records
//   [setup chain] -> [WHILE conditional node(body)]
// and `launch_sync()` replays it with a single launch. The loop body ends
// with a kernel that calls cudaGraphSetConditional(handle, keep_going) —
// no D2H/H2D anywhere in the loop.
//
// Availability: conditional nodes need toolkit AND driver >= 12.4;
// runtime_supported() checks the driver, the compile-time guard is
// CUDART_VERSION >= 12040 (capture() reports Unsupported otherwise).
// ---------------------------------------------------------------------------

#if CUDART_VERSION >= 12040
#define CUDA_TOOL_GRAPH_WHILE 1
#else
#define CUDA_TOOL_GRAPH_WHILE 0
#endif

namespace uipc::backend::cuda_tool
{
class GraphWhile
{
  public:
    enum class Result
    {
        Ok,
        Unsupported,  // toolkit or driver below CUDA 12.4
        CaptureFailed,
        BuildFailed,
        InstantiateFailed,
    };

    GraphWhile() = default;
    ~GraphWhile() { reset(); }

    GraphWhile(const GraphWhile&)            = delete;
    GraphWhile& operator=(const GraphWhile&) = delete;

    static bool runtime_supported()
    {
#if CUDA_TOOL_GRAPH_WHILE
        static int driver_version = []
        {
            int v = 0;
            cudaDriverGetVersion(&v);
            return v;
        }();
        return driver_version >= 12040;
#else
        return false;
#endif
    }

    // setup_body(stream, handle): ops run once per launch, before the loop.
    // loop_body(stream, handle): ONE loop iteration; its last kernel must
    // call cudaGraphSetConditional(handle, keep_going).
    // On failure the instance is permanently disabled.
    template <typename SetupF, typename BodyF>
    Result capture(SetupF&& setup_body, BodyF&& loop_body)
    {
        if(m_disabled)
            return m_failure;

#if CUDA_TOOL_GRAPH_WHILE
        reset_graph();
        if(!runtime_supported())
            return fail(Result::Unsupported);

        if(!m_capture_stream)
            // blocking on purpose: any callee launching into the legacy
            // default stream during capture then INVALIDATES the capture
            // (our automatic fallback signal). With a non-blocking capture
            // stream, such kernels would silently EXECUTE during capture and
            // be missing from the graph — the replayed graph would run the
            // solver with the preconditioner frozen at its capture-time
            // output (this produced false PCG convergence on MAS scenes).
            cudaStreamCreateWithFlags(&m_capture_stream, cudaStreamDefault);

        cudaError_t err;
        // 1) main graph + conditional handle first: both bodies bake the
        //    handle as a kernel argument, so it must exist before capture
        err = cudaGraphCreate(&m_graph, 0);
        if(err != cudaSuccess)
            return fail(Result::BuildFailed, "cudaGraphCreate", err);

        cudaGraphConditionalHandle handle;
        err = cudaGraphConditionalHandleCreate(&handle, m_graph, 1u, cudaGraphCondAssignDefault);
        if(err != cudaSuccess)
            return fail(Result::BuildFailed, "cudaGraphConditionalHandleCreate", err);

        // 2) setup chain (cloned into a child-graph node below)
        err = cudaStreamBeginCapture(m_capture_stream, cudaStreamCaptureModeThreadLocal);
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            return fail(Result::CaptureFailed);
        }
        setup_body(m_capture_stream, handle);
        err = cudaStreamEndCapture(m_capture_stream, &m_setup_graph);
        if(err != cudaSuccess || m_setup_graph == nullptr)
        {
            cudaGetLastError();
            m_setup_graph = nullptr;
            return fail(Result::CaptureFailed);
        }

        // 3) assemble: setup -> WHILE(body)
        cudaGraphNode_t setup_node;
        err = cudaGraphAddChildGraphNode(&setup_node, m_graph, nullptr, 0, m_setup_graph);
        if(err != cudaSuccess)
            return fail(Result::BuildFailed, "cudaGraphAddChildGraphNode(setup)", err);

        // NB: phGraph_out is an OUTPUT — the driver writes back a pointer to
        // its own body-graph array; do not point it at user storage
        cudaGraphNodeParams params = {};
        params.type                = cudaGraphNodeTypeConditional;
        params.conditional.handle  = handle;
        params.conditional.type    = cudaGraphCondTypeWhile;
        params.conditional.size    = 1;
        cudaGraphNode_t while_node;
#if CUDART_VERSION >= 13000
        err = cudaGraphAddNode(&while_node, m_graph, &setup_node, nullptr, 1, &params);
#else
        // CUDA 11.x-12.x: no edge-data parameter
        err = cudaGraphAddNode(&while_node, m_graph, &setup_node, 1, &params);
#endif
        cudaGraph_t body_container =
            params.conditional.phGraph_out ? params.conditional.phGraph_out[0] : nullptr;
        if(err != cudaSuccess || body_container == nullptr)
            return fail(Result::BuildFailed, "cudaGraphAddNode(conditional)", err);

        // 4) capture the loop body DIRECTLY into the conditional's body
        //    graph — a cloned child-graph node here would add one device-side
        //    dispatch level per iteration (measurably slower)
        err = cudaStreamBeginCaptureToGraph(
            m_capture_stream, body_container, nullptr, nullptr, 0, cudaStreamCaptureModeThreadLocal);
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            return fail(Result::CaptureFailed);
        }
        loop_body(m_capture_stream, handle);
        err = cudaStreamEndCapture(m_capture_stream, nullptr);
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            return fail(Result::CaptureFailed);
        }

        // 5) instantiate
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
#else
        return fail(Result::Unsupported);
#endif
    }

    cudaError_t launch_sync()
    {
        if(!m_launch_stream)
            cudaStreamCreateWithFlags(&m_launch_stream, cudaStreamDefault);
        cudaGraphLaunch(m_exec, m_launch_stream);
        return cudaStreamSynchronize(m_launch_stream);
    }

    bool               ready() const { return m_exec != nullptr; }
    bool               disabled() const { return m_disabled; }
    const std::string& failure_detail() const { return m_failure_detail; }

    void reset_graph()
    {
        if(m_exec)
        {
            cudaGraphExecDestroy(m_exec);
            m_exec = nullptr;
        }
        if(m_graph)
        {
            cudaGraphDestroy(m_graph);  // also destroys the driver-owned body graph
            m_graph = nullptr;
        }
        // the setup capture was cloned into a child node; the loop body was
        // captured directly into the driver-owned conditional body graph
        if(m_setup_graph)
        {
            cudaGraphDestroy(m_setup_graph);
            m_setup_graph = nullptr;
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
    Result fail(Result r, const char* what = "", cudaError_t err = cudaSuccess)
    {
        m_failure        = r;
        m_disabled       = true;
        m_failure_detail = std::string(what) + ": " + cudaGetErrorString(err);
        return r;
    }

    cudaStream_t    m_capture_stream = nullptr;
    cudaStream_t    m_launch_stream  = nullptr;
    cudaGraph_t     m_setup_graph    = nullptr;
    cudaGraph_t     m_graph          = nullptr;
    cudaGraphExec_t m_exec           = nullptr;
    bool            m_disabled       = false;
    Result          m_failure        = Result::Ok;
    std::string     m_failure_detail;
};
}  // namespace uipc::backend::cuda_tool
