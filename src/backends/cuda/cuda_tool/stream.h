/*
 * NVIDIA contributions: SideStreams, LazySideStream and SideStream fork/join
 * helpers.
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
#include <cuda_runtime.h>
// Provide a global-scope ::arg(std::complex) BEFORE any Eigen include. nvcc's
// passes cannot resolve Eigen MathFunctions.h's `using ::arg` otherwise. This must
// precede <uipc/common/type_define.h> (which pulls in <Eigen/Core>).
#if defined(__CUDACC__) && !defined(UIPC_EIGEN_ARG_SHIM)
#define UIPC_EIGEN_ARG_SHIM
#include <complex>
template <typename T>
__host__ __device__ inline T arg(const std::complex<T>& z)
{
    return std::atan2(std::imag(z), std::real(z));
}
#endif
#include <uipc/common/type_define.h>
#include <stdexcept>
#include <string>
#include <cstdio>
#include <span>
#include <vector>

namespace uipc::backend::cuda_tool
{
using std::span;

// host-side error string without pulling in fmt (avoids nvcc/UTF-8 issues)
inline std::string cuda_error_string(cudaError_t e, const char* expr, const char* file, int line)
{
    char buf[512];
    std::snprintf(buf,
                  sizeof(buf),
                  "[%s:%d] CUDA error %s (%d): %s",
                  file,
                  line,
                  expr,
                  static_cast<int>(e),
                  cudaGetErrorString(e));
    return buf;
}
}  // namespace uipc::backend::cuda_tool

namespace uipc::backend::cuda_tool
{
// Default stream used by all cuda_tool primitives when no stream is given.
// Kept identical to muda's default (the default stream) so behavior is unchanged.
inline cudaStream_t default_stream()
{
    return cudaStream_t{nullptr};
}

inline void check_cuda_error(cudaError_t e, const char* expr, const char* file, int line)
{
    if(e != cudaSuccess)
    {
        throw std::runtime_error{cuda_error_string(e, expr, file, line)};
    }
}

#define CUDA_TOOL_CHECK(EXPR)                                                  \
    ::uipc::backend::cuda_tool::check_cuda_error((EXPR), #EXPR, __FILE__, __LINE__)

// Block size used by cuda_tool launches (matches muda's default_parallel_dim).
inline constexpr int default_block_dim = 256;

// Lightweight stream handle wrapper (replacement for cuda_tool::Stream).
// The default stream is a process-wide singleton identical to cudaStream_t{nullptr}.
class Stream
{
  public:
    Stream() = default;
    explicit Stream(cudaStream_t h) noexcept
        : m_handle(h)
    {
    }
    operator cudaStream_t() const noexcept { return m_handle; }
    cudaStream_t view() const noexcept { return m_handle; }

    static Stream& Default() noexcept
    {
        static Stream s{nullptr};
        return s;
    }

  private:
    cudaStream_t m_handle = nullptr;
};

// Non-blocking side streams for fork/join concurrency next to an origin
// stream (normally the legacy default stream). They perform no implicit
// synchronization with the legacy stream: work on them is ordered only by
// fork() and join(), so a producer handed a side stream must launch all of
// its work on that stream.
class SideStreams
{
  public:
    SideStreams()                              = default;
    SideStreams(const SideStreams&)            = delete;
    SideStreams& operator=(const SideStreams&) = delete;
    ~SideStreams()
    {
        for(auto e : m_events)
            cudaEventDestroy(e);
        for(auto s : m_streams)
            cudaStreamDestroy(s);
        if(m_origin_event)
            cudaEventDestroy(m_origin_event);
    }

    // Create `count` streams once, at the greatest priority, so short
    // dependency chains on them are not queued behind wide kernels.
    void create(SizeT count)
    {
        if(!m_streams.empty() || count == 0)
            return;
        int least = 0, greatest = 0;
        CUDA_TOOL_CHECK(cudaDeviceGetStreamPriorityRange(&least, &greatest));
        CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_origin_event, cudaEventDisableTiming));
        m_streams.resize(count);
        m_events.resize(count);
        for(SizeT i = 0; i < count; ++i)
        {
            CUDA_TOOL_CHECK(cudaStreamCreateWithPriority(
                &m_streams[i], cudaStreamNonBlocking, greatest));
            CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_events[i], cudaEventDisableTiming));
        }
    }

    SizeT        size() const noexcept { return m_streams.size(); }
    cudaStream_t operator[](SizeT i) const noexcept { return m_streams[i]; }

    // Every side stream waits for the work enqueued on `origin` so far, and
    // the round-robin cursor of next() restarts.
    void fork(cudaStream_t origin = default_stream())
    {
        if(m_streams.empty())
            return;
        CUDA_TOOL_CHECK(cudaEventRecord(m_origin_event, origin));
        for(auto s : m_streams)
            CUDA_TOOL_CHECK(cudaStreamWaitEvent(s, m_origin_event, 0));
        m_next = 0;
    }

    // Side stream `i` alone waits for the work enqueued on `origin` so far.
    void fork_one(SizeT i, cudaStream_t origin = default_stream())
    {
        CUDA_TOOL_CHECK(cudaEventRecord(m_origin_event, origin));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(m_streams[i], m_origin_event, 0));
    }

    // Round-robin side stream for the next independent producer (the
    // default stream when none were created).
    cudaStream_t next() noexcept
    {
        if(m_streams.empty())
            return default_stream();
        return m_streams[m_next++ % m_streams.size()];
    }

    // `waiter` waits for the work enqueued on every side stream so far. A
    // side stream passed as `waiter` skips itself.
    void join(cudaStream_t waiter = default_stream())
    {
        for(SizeT i = 0; i < m_streams.size(); ++i)
        {
            if(m_streams[i] == waiter)
                continue;
            CUDA_TOOL_CHECK(cudaEventRecord(m_events[i], m_streams[i]));
            CUDA_TOOL_CHECK(cudaStreamWaitEvent(waiter, m_events[i], 0));
        }
    }

  private:
    std::vector<cudaStream_t> m_streams;
    std::vector<cudaEvent_t>  m_events;
    cudaEvent_t               m_origin_event = nullptr;
    SizeT                     m_next         = 0;
};

// Launch stream of one producer: the next side stream, taken on first use so
// producers that launch nothing do not occupy one, or the default stream
// without side streams. Not copyable: every launch of the producer must go
// to the same stream.
class LazySideStream
{
  public:
    explicit LazySideStream(SideStreams* side_streams = nullptr) noexcept
        : m_side_streams(side_streams)
    {
    }
    LazySideStream(const LazySideStream&)            = delete;
    LazySideStream& operator=(const LazySideStream&) = delete;

    cudaStream_t get() const noexcept
    {
        if(m_side_streams)
        {
            m_stream       = m_side_streams->next();
            m_side_streams = nullptr;
        }
        return m_stream;
    }

  private:
    mutable SideStreams* m_side_streams = nullptr;
    mutable cudaStream_t m_stream       = default_stream();
};

// Owning non-blocking stream for work that runs beside another stream. It
// does not synchronize implicitly with the legacy default stream: fork()
// orders it after the work enqueued on the parent so far, join() orders the
// parent after the work enqueued on it so far. Everything it reads or writes
// must be enqueued between the two.
class SideStream
{
  public:
    SideStream() = default;
    ~SideStream()
    {
        if(m_join)
            cudaEventDestroy(m_join);
        if(m_fork)
            cudaEventDestroy(m_fork);
        if(m_stream)
            cudaStreamDestroy(m_stream);
    }
    SideStream(const SideStream&)            = delete;
    SideStream& operator=(const SideStream&) = delete;

    cudaStream_t fork(cudaStream_t parent = default_stream())
    {
        if(!m_stream)
        {
            CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&m_stream, cudaStreamNonBlocking));
            CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_fork, cudaEventDisableTiming));
            CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_join, cudaEventDisableTiming));
        }
        CUDA_TOOL_CHECK(cudaEventRecord(m_fork, parent));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(m_stream, m_fork, 0));
        return m_stream;
    }

    void join(cudaStream_t parent = default_stream())
    {
        CUDA_TOOL_CHECK(cudaEventRecord(m_join, m_stream));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(parent, m_join, 0));
    }

  private:
    cudaStream_t m_stream = nullptr;
    cudaEvent_t  m_fork   = nullptr;
    cudaEvent_t  m_join   = nullptr;
};
}  // namespace uipc::backend::cuda_tool
