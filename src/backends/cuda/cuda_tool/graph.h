/*
 * NVIDIA contributions: non-blocking graph launches, in-place recapture, the
 * WHILE-loop launch, the invalidated-capture fallback and shared stream and
 * instantiation helpers.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
//    invalidates the capture (reported as Result::CaptureInvalidated). So
//    does a body that throws after such an invalidation (a later checked
//    call on a captured stream fails, e.g. SideStreams::join's event
//    record): capture() ends the capture first, so no stream is left
//    capturing. Other exceptions propagate once the capture is ended.
//  - kernel arguments are baked at capture time, including by-value counts
//    inside views — the caller must key validity on anything that can change.
//  - no host-side reads of the captured region's outputs during capture.
// ---------------------------------------------------------------------------
#include <cuda_runtime.h>
#include <string>
#include <utility>

namespace uipc::backend::cuda_tool
{
namespace details
{
    // Capture and launch streams are blocking on purpose. A callee that
    // launches into the legacy default stream during a capture then
    // INVALIDATES the capture (the automatic fallback signal); with a
    // non-blocking capture stream such kernels would silently EXECUTE during
    // capture and be missing from the graph, and the replayed graph would
    // run the solver with the preconditioner frozen at its capture-time
    // output (this produced false PCG convergence on MAS scenes). A replay
    // on a blocking launch stream is ordered with the legacy stream.
    inline void create_blocking_stream(cudaStream_t& stream)
    {
        if(!stream)
            cudaStreamCreateWithFlags(&stream, cudaStreamDefault);
    }

    inline void destroy_stream(cudaStream_t& stream)
    {
        if(stream)
        {
            cudaStreamDestroy(stream);
            stream = nullptr;
        }
    }

    // cudaGraphInstantiateWithFlags from CUDA 11.4 on, the legacy 5-argument
    // cudaGraphInstantiate before.
    inline cudaError_t instantiate(cudaGraphExec_t* exec, cudaGraph_t graph)
    {
#if CUDART_VERSION >= 11040
        return cudaGraphInstantiateWithFlags(exec, graph, 0);
#else
        return cudaGraphInstantiate(exec, graph, nullptr, nullptr, 0);
#endif
    }

    // Runs a capture body on `stream`. If it throws, the capture is ended
    // here, which takes every stream that joined it out of capture mode.
    // Returns false when the capture had been invalidated before the throw:
    // a callee escaped the capture stream and a later checked call on a
    // captured stream failed. The caller then reports a failed capture, as
    // when cudaStreamEndCapture detects the invalidation. Other exceptions
    // are rethrown. `graph` receives the graph cudaStreamEndCapture returns
    // (nullptr for a capture into an existing graph).
    template <typename F>
    bool run_capture_body(cudaStream_t stream, cudaGraph_t* graph, F&& body)
    {
        try
        {
            body();
            return true;
        }
        catch(...)
        {
            cudaStreamCaptureStatus status = cudaStreamCaptureStatusNone;
            if(cudaStreamIsCapturing(stream, &status) != cudaSuccess)
                status = cudaStreamCaptureStatusActive;  // unknown: end it anyway
            if(status != cudaStreamCaptureStatusNone)
                cudaStreamEndCapture(stream, graph);
            cudaGetLastError();
            if(status != cudaStreamCaptureStatusInvalidated)
                throw;
            return false;
        }
    }
}  // namespace details

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

        details::create_blocking_stream(m_capture_stream);

        cudaError_t err =
            cudaStreamBeginCapture(m_capture_stream, cudaStreamCaptureModeThreadLocal);
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            return fail(Result::BeginCaptureFailed);
        }

        if(!details::run_capture_body(
               m_capture_stream, &m_graph, [&] { body(m_capture_stream); }))
            return fail(Result::CaptureInvalidated);

        err = cudaStreamEndCapture(m_capture_stream, &m_graph);
        if(err != cudaSuccess || m_graph == nullptr)
        {
            cudaGetLastError();
            m_graph = nullptr;
            return fail(Result::CaptureInvalidated);
        }

        err = details::instantiate(&m_exec, m_graph);
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            reset_graph();
            return fail(Result::InstantiateFailed);
        }
        return Result::Ok;
    }

    // capture() for a graph whose launches may have changed: a new capture
    // updates the instantiated graph in place (cudaGraphExecUpdate) when it
    // has the same topology, which costs less than a new instantiation, and
    // is instantiated otherwise. Without an instantiated graph it is
    // capture(). Failures disable the instance as in capture(), and a body
    // that throws on a valid capture leaves it as capture() does.
    template <typename F>
    Result recapture(F&& body)
    {
        if(m_disabled || !m_exec)
            return capture(std::forward<F>(body));

        cudaGraph_t graph = nullptr;
        cudaError_t err =
            cudaStreamBeginCapture(m_capture_stream, cudaStreamCaptureModeThreadLocal);
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            reset_graph();
            return fail(Result::BeginCaptureFailed);
        }
        bool captured = false;
        try
        {
            captured = details::run_capture_body(
                m_capture_stream, &graph, [&] { body(m_capture_stream); });
        }
        catch(...)
        {
            // As in capture(): the instance keeps the graph of the ended
            // capture (reset() frees it) and no instantiated graph, so
            // ready() turns false and the stale graph cannot be replayed.
            reset_graph();
            m_graph = graph;
            throw;
        }
        if(!captured)
        {
            if(graph)
                cudaGraphDestroy(graph);
            reset_graph();
            return fail(Result::CaptureInvalidated);
        }
        err = cudaStreamEndCapture(m_capture_stream, &graph);
        if(err != cudaSuccess || graph == nullptr)
        {
            cudaGetLastError();
            reset_graph();
            return fail(Result::CaptureInvalidated);
        }

#if CUDART_VERSION >= 12000
        cudaGraphExecUpdateResultInfo info;
        if(cudaGraphExecUpdate(m_exec, graph, &info) == cudaSuccess)
        {
            cudaGraphDestroy(m_graph);
            m_graph = graph;
            return Result::Ok;
        }
        cudaGetLastError();
#endif
        reset_graph();
        m_graph = graph;
        err     = details::instantiate(&m_exec, m_graph);
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
        details::create_blocking_stream(m_launch_stream);
        cudaGraphLaunch(m_exec, m_launch_stream);
        cudaStreamSynchronize(m_launch_stream);
    }

    // Enqueue the captured graph on `stream` without waiting for it: it runs
    // after the work enqueued on `stream` so far, and work enqueued on
    // `stream` afterwards runs after it.
    cudaError_t launch(cudaStream_t stream)
    {
        return cudaGraphLaunch(m_exec, stream);
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
        details::destroy_stream(m_capture_stream);
        details::destroy_stream(m_launch_stream);
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

        details::create_blocking_stream(m_capture_stream);

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
        if(!details::run_capture_body(m_capture_stream,
                                      &m_setup_graph,
                                      [&]
                                      { setup_body(m_capture_stream, handle); }))
            return fail(Result::CaptureFailed,
                        "setup chain threw on the invalidated capture",
                        cudaErrorStreamCaptureInvalidated);
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
        // A failed capture into the conditional body leaves the main graph
        // unsafe to destroy (CUDA 13.4: the body graph handle turns invalid
        // and cudaGraphDestroy of the main graph crashes), so it is
        // abandoned; this happens once, as the instance is then disabled.
        if(!details::run_capture_body(m_capture_stream,
                                      nullptr,
                                      [&]
                                      { loop_body(m_capture_stream, handle); }))
        {
            m_graph = nullptr;
            return fail(Result::CaptureFailed,
                        "loop body threw on the invalidated capture",
                        cudaErrorStreamCaptureInvalidated);
        }
        err = cudaStreamEndCapture(m_capture_stream, nullptr);
        if(err != cudaSuccess)
        {
            cudaGetLastError();
            m_graph = nullptr;
            return fail(Result::CaptureFailed, "cudaStreamEndCapture(loop body)", err);
        }

        // 5) instantiate
        err = details::instantiate(&m_exec, m_graph);
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

    // Enqueue the graph on the internal blocking stream without waiting for
    // it: work enqueued afterwards on the legacy default stream waits for it,
    // the host does not.
    cudaError_t launch()
    {
        details::create_blocking_stream(m_launch_stream);
        return cudaGraphLaunch(m_exec, m_launch_stream);
    }

    // Stream of launch() (null before the first launch).
    cudaStream_t launch_stream() const { return m_launch_stream; }

    cudaError_t launch_sync()
    {
        cudaError_t err = launch();
        if(err != cudaSuccess)
            return err;
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
        details::destroy_stream(m_capture_stream);
        details::destroy_stream(m_launch_stream);
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
